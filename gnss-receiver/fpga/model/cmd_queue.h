/*
 * The per-channel queue of tagged NCO commands (core/include/gnss/corr_if.h),
 * shared by the bit-exact model and the float correlator so the two cannot
 * drift apart. The HDL implements the same rules.
 */
#ifndef GNSS_FPGA_CMD_QUEUE_H
#define GNSS_FPGA_CMD_QUEUE_H

#include <stdint.h>

#include "gnss/corr_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t valid;
    uint32_t apply_seq;
    int32_t carr_word;
    uint64_t code_word;
} cmdq_entry_t;

typedef struct {
    cmdq_entry_t e[CORR_CMD_QUEUE];
    uint8_t flags;              /* CORR_DUMP_DROPPED, reported with the next dump */
} cmdq_t;

static inline void cmdq_clear(cmdq_t *q)
{
    for (int k = 0; k < CORR_CMD_QUEUE; k++) {
        q->e[k].valid = 0;
    }
    q->flags = 0;
}

/* Queues a command: same tag replaces; else a free slot; else replace the later-tagged entry. */
static inline void cmdq_push(cmdq_t *q, uint32_t apply_seq, int32_t carr_word, uint64_t code_word)
{
    int slot = -1;
    for (int k = 0; k < CORR_CMD_QUEUE; k++) {
        if (q->e[k].valid && q->e[k].apply_seq == apply_seq) {
            slot = k;
        }
    }
    for (int k = 0; slot < 0 && k < CORR_CMD_QUEUE; k++) {
        if (!q->e[k].valid) {
            slot = k;
        }
    }
    if (slot < 0) {
        slot = 0;
        for (int k = 1; k < CORR_CMD_QUEUE; k++) {
            if (q->e[k].apply_seq > q->e[slot].apply_seq) {
                slot = k;
            }
        }
        q->flags |= CORR_DUMP_DROPPED;
    }
    q->e[slot].valid = 1;
    q->e[slot].apply_seq = apply_seq;
    q->e[slot].carr_word = carr_word;
    q->e[slot].code_word = code_word;
}

/*
 * At the epoch closing period seq: applies the latest entry whose tag has come
 * (writing *carr_word, *code_word), drops older ones, and returns the dump flags
 * (CORR_DUMP_LATE if the applied tag was earlier than seq).
 */
static inline uint8_t cmdq_epoch(cmdq_t *q, uint32_t seq, int32_t *carr_word, uint64_t *code_word)
{
    int best = -1;
    for (int k = 0; k < CORR_CMD_QUEUE; k++) {
        if (q->e[k].valid && q->e[k].apply_seq <= seq && (best < 0 || q->e[k].apply_seq > q->e[best].apply_seq)) {
            best = k;
        }
    }
    uint8_t flags = q->flags;
    q->flags = 0;
    if (best >= 0) {
        *carr_word = q->e[best].carr_word;
        *code_word = q->e[best].code_word;
        if (q->e[best].apply_seq < seq) {
            flags |= CORR_DUMP_LATE;
        }
        for (int k = 0; k < CORR_CMD_QUEUE; k++) {
            if (q->e[k].valid && q->e[k].apply_seq <= seq) {
                q->e[k].valid = 0;
            }
        }
    }
    return flags;
}

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FPGA_CMD_QUEUE_H */
