/* Signal definitions and spreading codes. */
#ifndef GNSS_SIG_H
#define GNSS_SIG_H

#include <stdint.h>

#include "gnss/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_C 299792458.0          /* speed of light, m/s (IS-GPS-200) */

typedef struct {
    gnss_sig_t sig;
    gnss_sys_t sys;
    gnss_band_t band;
    double carrier_hz;
    double chip_rate;               /* chips per second */
    uint16_t code_len;              /* chips per primary code period */
    uint16_t symbol_ms;             /* data symbol length (0 for a pilot) */
    uint8_t boc11;                  /* BOC(1,1) subcarrier */
} gnss_sigdef_t;

/* NULL for a signal not defined yet. */
const gnss_sigdef_t *gnss_sigdef(gnss_sig_t sig);

#define GPS_CA_LEN 1023
#define GPS_MAX_PRN 32

/* The PRN's C/A code (IS-GPS-200 G1/G2 Gold code), chip 0 first, as 0/1. Returns -1 for a bad PRN. */
int gps_ca_code(int prn, uint8_t chips[GPS_CA_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_SIG_H */
