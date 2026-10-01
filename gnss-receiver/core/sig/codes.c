#include "gnss/sig.h"

#include <stddef.h>

static const gnss_sigdef_t kSigs[] = {
    {GNSS_SIG_GPS_L1CA, GNSS_SYS_GPS, GNSS_BAND_L1, GNSS_FREQ_L1_HZ, 1.023e6, GPS_CA_LEN, 20, 0},
};

const gnss_sigdef_t *gnss_sigdef(gnss_sig_t sig)
{
    for (size_t i = 0; i < sizeof(kSigs) / sizeof(kSigs[0]); i++) {
        if (kSigs[i].sig == sig) {
            return &kSigs[i];
        }
    }
    return NULL;
}

/* G2 phase-selector taps, IS-GPS-200 Table 3-Ia. */
static const uint8_t kG2Taps[GPS_MAX_PRN][2] = {
    {2, 6}, {3, 7}, {4, 8}, {5, 9}, {1, 9}, {2, 10}, {1, 8}, {2, 9},
    {3, 10}, {2, 3}, {3, 4}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {9, 10},
    {1, 4}, {2, 5}, {3, 6}, {4, 7}, {5, 8}, {6, 9}, {1, 3}, {4, 6},
    {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 6}, {2, 7}, {3, 8}, {4, 9},
};

int gps_ca_code(int prn, uint8_t chips[GPS_CA_LEN])
{
    if (prn < 1 || prn > GPS_MAX_PRN) {
        return -1;
    }
    /* Stage s of each register is g[s - 1]; both start all ones. */
    uint8_t g1[10], g2[10];
    for (int s = 0; s < 10; s++) {
        g1[s] = 1;
        g2[s] = 1;
    }
    int t1 = kG2Taps[prn - 1][0] - 1, t2 = kG2Taps[prn - 1][1] - 1;
    for (int i = 0; i < GPS_CA_LEN; i++) {
        chips[i] = (uint8_t)(g1[9] ^ g2[t1] ^ g2[t2]);
        uint8_t f1 = g1[2] ^ g1[9];                                       /* 1 + x^3 + x^10 */
        uint8_t f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];       /* 1 + x^2 + x^3 + x^6 + x^8 + x^9 + x^10 */
        for (int s = 9; s > 0; s--) {
            g1[s] = g1[s - 1];
            g2[s] = g2[s - 1];
        }
        g1[0] = f1;
        g2[0] = f2;
    }
    return 0;
}
