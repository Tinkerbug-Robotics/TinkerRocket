/*
 * The correlator interface: what the P4 writes to the FPGA's channels and what
 * it reads back. On the host the float correlator (host/corr_float.c) and,
 * from milestone 4, the bit-exact model (fpga/model/) sit behind it; on the
 * target it is the QSPI register map.
 *
 * Timing contract:
 * - Time is the sample count at the correlator's rate (fs), from the stream's start.
 * - START takes effect at sample t_start. The channel's first dump covers a partial
 *   code period and has seq 0.
 * - A channel dumps at every code epoch: the sample where its prompt code phase
 *   wraps. The dump holds the sums over the period that just ended, and the NCO
 *   state at the epoch's first sample (t_samp).
 * - NCO commands are latched and take effect at the channel's next epoch. A
 *   command written after dump k therefore steers the period after the one in
 *   progress (the one-period delay the loop design accounts for).
 * - The P4 is interrupted every 1 ms (the tick) and reads every dump made since
 *   the last tick, in order.
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
    uint64_t t_start;       /* START: sample at which the channel starts */
    uint64_t code_phase;    /* START: prompt code phase at t_start, chips << CORR_CODE_FRAC_BITS */
    uint64_t tap_offset;    /* START: early/late offset from prompt, same units */
    int32_t carr_word;      /* START, NCO: carrier increment per sample, cycles * 2^32 */
    uint64_t code_word;     /* START, NCO: code increment per sample, chips << CORR_CODE_FRAC_BITS */
} corr_cmd_t;

typedef struct {
    uint8_t ch;
    uint32_t seq;           /* dumps since START; seq 0 covers a partial period */
    uint64_t t_samp;        /* first sample of the new code period */
    uint64_t code_phase;    /* prompt code phase at t_samp (a small fraction of a chip past the epoch) */
    uint32_t carr_phase;    /* carrier NCO phase at t_samp, cycles * 2^32 */
    uint32_t carr_cycles;   /* whole carrier cycles since START at t_samp (wraps) */
    int32_t carr_word;      /* the words in force during the period */
    uint64_t code_word;
    float ie, qe, ip, qp, il, ql;  /* early, prompt, late sums over the period */
} corr_dump_t;

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CORR_IF_H */
