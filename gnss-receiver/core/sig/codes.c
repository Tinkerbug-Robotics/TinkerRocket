#include "gnss/sig.h"

#include "b1c_params.h"
#include "gal_e1_codes.h"

#include <stddef.h>

static const gnss_sigdef_t kSigs[] = {
    {GNSS_SIG_GPS_L1CA, GNSS_SYS_GPS, GNSS_BAND_L1, GNSS_FREQ_L1_HZ, 1.023e6, GPS_CA_LEN, 20, 0},
    {GNSS_SIG_GAL_E1B, GNSS_SYS_GAL, GNSS_BAND_L1, GNSS_FREQ_L1_HZ, 1.023e6, GAL_E1_LEN, 4, 1},
    {GNSS_SIG_GAL_E1C, GNSS_SYS_GAL, GNSS_BAND_L1, GNSS_FREQ_L1_HZ, 1.023e6, GAL_E1_LEN, 0, 1},
    {GNSS_SIG_BDS_B1CD, GNSS_SYS_BDS, GNSS_BAND_L1, GNSS_FREQ_L1_HZ, 1.023e6, BDS_B1C_LEN, 10, 1},
    {GNSS_SIG_BDS_B1CP, GNSS_SYS_BDS, GNSS_BAND_L1, GNSS_FREQ_L1_HZ, 1.023e6, BDS_B1C_LEN, 0, 1},
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

int gal_e1_code(int prn, gnss_sig_t sig, uint8_t chips[GAL_E1_LEN])
{
    if (prn < 1 || prn > GAL_MAX_PRN || (sig != GNSS_SIG_GAL_E1B && sig != GNSS_SIG_GAL_E1C)) {
        return -1;
    }
    const uint8_t *b = (sig == GNSS_SIG_GAL_E1B) ? gal_e1b_codes[prn - 1] : gal_e1c_codes[prn - 1];
    for (int i = 0; i < GAL_E1_LEN; i++) {
        chips[i] = (uint8_t)((b[i >> 3] >> (7 - (i & 7))) & 1u);
    }
    return 0;
}

void gal_e1c_secondary(uint8_t chips[GAL_E1C_SEC_LEN])
{
    /* CS25_1 = 380AD90 (hex, OS SIS ICD Table 20): 25 chips, then 3 zeros of padding. */
    const uint32_t cs25 = 0x380AD90u >> 3;
    for (int i = 0; i < GAL_E1C_SEC_LEN; i++) {
        chips[i] = (uint8_t)((cs25 >> (GAL_E1C_SEC_LEN - 1 - i)) & 1u);
    }
}

/* Legendre symbol bit of k modulo the prime n (BDS-SIS-ICD-B1C 5-2): 1 if k is a nonzero
 * quadratic residue, by Euler's criterion k^((n-1)/2) = 1 (mod n). */
static uint8_t legendre_bit(uint32_t k, uint32_t n)
{
    if (k % n == 0) {
        return 0;
    }
    uint32_t r = 1, b = k % n, e = (n - 1) / 2;
    while (e) {
        if (e & 1u) {
            r = (r * b) % n;
        }
        b = (b * b) % n;
        e >>= 1;
    }
    return r == 1;
}

/* c(i) = W((i + p - 1) mod n; w), with W(k; w) = L(k) xor L((k + w) mod n) (5-1, 5-3). */
static void weil(uint32_t n, uint32_t w, uint32_t p, uint8_t *chips, int len)
{
    for (int i = 0; i < len; i++) {
        uint32_t k = ((uint32_t)i + p - 1u) % n;
        chips[i] = (uint8_t)(legendre_bit(k, n) ^ legendre_bit((k + w) % n, n));
    }
}

int bds_b1c_code(int prn, gnss_sig_t sig, uint8_t chips[BDS_B1C_LEN])
{
    if (prn < 1 || prn > BDS_MAX_PRN || (sig != GNSS_SIG_BDS_B1CD && sig != GNSS_SIG_BDS_B1CP)) {
        return -1;
    }
    const uint16_t *wp = (sig == GNSS_SIG_BDS_B1CD) ? b1c_data_wp[prn - 1] : b1c_pilot_wp[prn - 1];
    weil(10243u, wp[0], wp[1], chips, BDS_B1C_LEN);
    return 0;
}

int bds_b1c_secondary(int prn, uint8_t chips[BDS_B1C_SEC_LEN])
{
    if (prn < 1 || prn > BDS_MAX_PRN) {
        return -1;
    }
    weil(3607u, b1c_secondary_wp[prn - 1][0], b1c_secondary_wp[prn - 1][1], chips, BDS_B1C_SEC_LEN);
    return 0;
}
