/*
 * The correlator interface: what the P4 writes to the FPGA's channels and what
 * it reads back. On the host the float correlator (host/corr_float.c) and,
 * from milestone 4, the bit-exact model (fpga/model/) sit behind it; on the
 * target it is the QSPI register map.
 *
 * Timing contract:
 * - Time is the sample count at the correlator's rate (fs), from the stream's start,
 *   CORR_TSAMP_BITS wide. Period counts (seq, apply_seq) are CORR_SEQ_BITS wide. Both
 *   wrap, and are compared as signed differences modulo their width
 *   (corr_tsamp_diff, corr_seq_diff).
 * - START takes effect at sample t_start, or at once if that sample has passed. The
 *   channel's first dump covers a partial code period and has seq 0.
 * - A channel dumps at every code epoch: the sample where its prompt code phase
 *   wraps. The dump holds the sums over the period that just ended, and the NCO
 *   state at the epoch's first sample (t_samp).
 * - NCO commands are tagged (owner, 2026-09-30). A command's apply_seq names
 *   the period whose closing epoch switches the channel to its words: they
 *   govern period apply_seq + 1 onward. Each channel holds CORR_CMD_QUEUE
 *   pending commands. At an epoch it applies the latest one whose tag has
 *   come, and drops any older ones. A command applied after its tagged epoch
 *   (it arrived late) sets CORR_DUMP_LATE in that epoch's dump. A command that
 *   finds the queue full replaces the pending one with the later tag and sets
 *   CORR_DUMP_DROPPED.
 * - The P4 is interrupted every 1 ms (the tick) and reads every dump made since
 *   the last tick, in order. It tags a command computed from dump s with
 *   s + 2. The tick is not aligned to any channel's epochs, so that leaves at
 *   least one full period for its own latency. The delay from a measurement to
 *   its correction is then fixed: period s steers period s + 3, whatever the
 *   tick phase or the P4's processing time.
 */
#ifndef GNSS_CORR_IF_H
#define GNSS_CORR_IF_H

#include <stdint.h>

#include "gnss/corr_params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CORR_CMD_START = 1,
    CORR_CMD_NCO   = 2,
    CORR_CMD_STOP  = 3
} corr_cmd_type_t;

typedef struct {
    uint8_t type;           /* corr_cmd_type_t */
    uint8_t ch;
    uint8_t sig;            /* gnss_sig_t (START) */
    uint8_t prn;            /* (START) */
    uint64_t t_start;       /* START: sample at which the channel starts (CORR_TSAMP_BITS) */
    uint64_t code_phase;    /* START: prompt code phase at t_start, chips << CORR_CODE_FRAC_BITS */
    uint64_t tap_offset;    /* START: early/late offset from prompt, same units */
    int32_t carr_word;      /* START, NCO: carrier increment per sample, cycles * 2^32 */
    uint64_t code_word;     /* START, NCO: code increment per sample, chips << CORR_CODE_FRAC_BITS */
    uint32_t apply_seq;     /* NCO: the period whose closing epoch switches to these words (CORR_SEQ_BITS) */
} corr_cmd_t;

#define CORR_TSAMP_MASK ((UINT64_C(1) << CORR_TSAMP_BITS) - 1u)
#define CORR_SEQ_MASK   ((UINT32_C(1) << CORR_SEQ_BITS) - 1u)

/* a - b for two sample counts, modulo CORR_TSAMP_BITS, as a signed number. */
static inline int64_t corr_tsamp_diff(uint64_t a, uint64_t b)
{
    uint64_t d = (a - b) & CORR_TSAMP_MASK;
    return d >> (CORR_TSAMP_BITS - 1) ? (int64_t)d - (int64_t)(UINT64_C(1) << CORR_TSAMP_BITS) : (int64_t)d;
}

/* a - b for two period counts, modulo CORR_SEQ_BITS, as a signed number. */
static inline int32_t corr_seq_diff(uint32_t a, uint32_t b)
{
    uint32_t d = (a - b) & CORR_SEQ_MASK;
    return d >> (CORR_SEQ_BITS - 1) ? (int32_t)d - (int32_t)(UINT32_C(1) << CORR_SEQ_BITS) : (int32_t)d;
}

#define CORR_DUMP_LATE    0x01u  /* a command took effect at this epoch, after the one its tag named */
#define CORR_DUMP_DROPPED 0x02u  /* a command found the queue full since the last dump */

typedef struct {
    uint8_t ch;
    uint32_t seq;           /* dumps since START, CORR_SEQ_BITS (wraps); seq 0 covers a partial period */
    uint64_t t_samp;        /* first sample of the new code period, CORR_TSAMP_BITS (wraps) */
    uint64_t code_phase;    /* prompt code phase at t_samp (a small fraction of a chip past the epoch) */
    uint32_t carr_phase;    /* carrier NCO phase at t_samp, cycles * 2^32 */
    uint32_t carr_cycles;   /* whole carrier cycles since START at t_samp (wraps) */
    int32_t carr_word;      /* the words in force during the period */
    uint64_t code_word;
    uint8_t flags;          /* CORR_DUMP_* */
    float ie, qe, ip, qp, il, ql;  /* early, prompt, late sums over the period */
} corr_dump_t;

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CORR_IF_H */
