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

#define GAL_E1_LEN 4092
#define GAL_MAX_PRN 50
#define GAL_E1C_SEC_LEN 25

/*
 * Galileo E1-B (sig GNSS_SIG_GAL_E1B) or E1-C (GNSS_SIG_GAL_E1C) primary code, chip 0 first,
 * as the ICD's logic 0/1: memory codes from the OS SIS ICD (core/sig/gal_e1_codes.c, which is
 * under the EU's terms, not this repository's licence). Returns -1 for a bad PRN or signal.
 */
int gal_e1_code(int prn, gnss_sig_t sig, uint8_t chips[GAL_E1_LEN]);

/* E1-C's secondary code CS25_1, the same for every satellite, as 0/1. */
void gal_e1c_secondary(uint8_t chips[GAL_E1C_SEC_LEN]);

#define BDS_B1C_LEN 10230
#define BDS_MAX_PRN 63
#define BDS_B1C_SEC_LEN 1800

/*
 * BeiDou B1C data (GNSS_SIG_BDS_B1CD) or pilot (GNSS_SIG_BDS_B1CP) primary code, as 0/1: a
 * Weil code of length 10243 truncated to 10230 (BDS-SIS-ICD-B1C-1.0, 5.2.1), generated here from
 * the ICD's parameters. Returns -1 for a bad PRN or signal.
 */
int bds_b1c_code(int prn, gnss_sig_t sig, uint8_t chips[BDS_B1C_LEN]);

/* The B1C pilot's secondary code: a Weil code of length 3607 truncated to 1800 (5.2.2). */
int bds_b1c_secondary(int prn, uint8_t chips[BDS_B1C_SEC_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_SIG_H */
