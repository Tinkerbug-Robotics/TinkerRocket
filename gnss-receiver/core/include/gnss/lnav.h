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
    int week;                        /* from subframe 1, full week; -1 until known */
    /* Ephemeris assembly. */
    uint32_t sf_data[3][10];         /* data bits of subframes 1-3, 24 per word */
    int have_sf;                     /* bit k set: subframe k+1 held */
    gps_eph_t eph;                   /* the latest complete ephemeris */
    gps_iono_t iono;
    uint32_t n_subframes, n_parity_fail;
} lnav_t;

void lnav_init(lnav_t *l, int prn);

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
