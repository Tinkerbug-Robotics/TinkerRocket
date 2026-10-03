/*
 * GPS LNAV (IS-GPS-200 section 20.3): frame sync on the preamble, word parity,
 * subframes 1-3 (ephemeris and clock) and subframe 4 page 18 (ionosphere).
 * One decoder per channel; it is fed one data bit at a time.
 */
#ifndef GNSS_LNAV_H
#define GNSS_LNAV_H

#include <stdint.h>

#include "gnss/eph.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LNAV_SF_BITS 300

typedef struct {
    int prn;
    uint8_t bits[LNAV_SF_BITS + 2];  /* the last 302 bits, oldest first (0/1) */
    uint32_t period[LNAV_SF_BITS + 2];  /* first code period of each */
    int nbits;                       /* valid entries, up to 302 */
    int synced;                      /* a subframe has passed parity */
    int inverted;                    /* the bit stream arrives inverted (Costas half-cycle) */
    /* Timing: the subframe that most recently passed parity started (its first bit's first code
     * period) at sf_period, and was transmitted at sf_tow (s of week). */
    uint32_t sf_period;
    double sf_tow;
    /* A HOW not yet confirmed: the timing counts only once the next subframe's HOW lands 6000 code
     * periods later with the count one up. Two real words in a row always pass parity, and the
     * almanac pages are the same on every satellite, so one HOW alone can be words 9-10 of a page
     * that happens to begin like a preamble -- on all satellites at once. */
    uint32_t cand_period, cand_tow;
    int cand_valid, cand_inv;
    /* A whole subframe that passed parity before its timing was confirmed: its data waits for the
     * confirmation (start period, ID, words). */
    uint32_t pend_period, pend_dw[10];
    int pend_id, pend_inv;
    int ready_id;                    /* a held subframe the latest confirmation released */
    int week;                        /* from subframe 1, full week; -1 until known */
    int week_ref;                    /* a full week near the true one: the 10-bit week resolves to the
                                        nearest (LNAV_WEEK_REF, unless the receiver knows better) */
    /* Ephemeris assembly. */
    uint32_t sf_data[3][10];         /* data bits of subframes 1-3, 24 per word */
    int have_sf;                     /* bit k set: subframe k+1 held */
    gps_eph_t eph;                   /* the latest complete ephemeris */
    gps_iono_t iono;
    uint32_t n_subframes, n_parity_fail;
} lnav_t;

/* The default week reference: 10-bit weeks resolve into 2048-3071 (2019-04-07 to 2038-11-20). */
#define LNAV_WEEK_REF 2560

void lnav_init(lnav_t *l, int prn);

/* The full GPS week in [ref - 512, ref + 512) whose low 10 bits are wn10. */
int lnav_resolve_week(int wn10, int ref);

/*
 * Pushes a data bit (+1/-1) whose first code period is `period`. Returns the ID
 * (1-5) of a subframe completed by this bit, 0 otherwise.
 */
int lnav_push(lnav_t *l, int bit, uint32_t period);

/*
 * Checks one 30-bit word (bit 29 = d1 ... bit 0 = D30) given the previous
 * word's D29 and D30. On success writes the 24 data bits (d1 at bit 23),
 * un-inverted, and returns 1.
 */
int lnav_parity(uint32_t word, int d29s, int d30s, uint32_t *data);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_LNAV_H */
