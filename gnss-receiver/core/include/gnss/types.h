/*
 * System, band and signal identifiers. Every structure that carries a
 * measurement carries a signal ID, and every signal maps to a band, so an L5
 * front end can be added without changing a message layout.
 */
#ifndef GNSS_TYPES_H
#define GNSS_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GNSS_SYS_GPS = 0,
    GNSS_SYS_GAL = 1,
    GNSS_SYS_BDS = 2,
    GNSS_SYS_QZS = 3,
    GNSS_SYS_COUNT
} gnss_sys_t;

/* One band per front-end chip. */
typedef enum {
    GNSS_BAND_L1 = 0,   /* 1575.42 MHz: GPS L1, Galileo E1, BeiDou B1C, QZSS L1 */
    GNSS_BAND_L5 = 1,   /* 1176.45 MHz: GPS L5, Galileo E5a, BeiDou B2a (later) */
    GNSS_BAND_COUNT
} gnss_band_t;

typedef enum {
    GNSS_SIG_GPS_L1CA = 0,
    GNSS_SIG_GAL_E1B  = 1,  /* data */
    GNSS_SIG_GAL_E1C  = 2,  /* pilot */
    GNSS_SIG_BDS_B1CD = 3,  /* data */
    GNSS_SIG_BDS_B1CP = 4,  /* pilot */
    GNSS_SIG_GPS_L1CD = 5,  /* later */
    GNSS_SIG_GPS_L1CP = 6,  /* later */
    GNSS_SIG_GPS_L5I  = 7,  /* L5 band, later */
    GNSS_SIG_GPS_L5Q  = 8,
    GNSS_SIG_GAL_E5AI = 9,
    GNSS_SIG_GAL_E5AQ = 10,
    GNSS_SIG_BDS_B2AD = 11,
    GNSS_SIG_BDS_B2AP = 12,
    GNSS_SIG_COUNT
} gnss_sig_t;

#define GNSS_FREQ_L1_HZ 1575420000.0
#define GNSS_FREQ_L5_HZ 1176450000.0

#ifdef __cplusplus
}
#endif

#endif /* GNSS_TYPES_H */
