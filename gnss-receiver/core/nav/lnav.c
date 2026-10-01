#include "gnss/lnav.h"

#include <string.h>

#define B(i) (1u << (24 - (i)))  /* data bit d_i of a 24-bit word, d1 the MSB */

/* IS-GPS-200 Table 20-XIV: the data bits each parity bit covers. */
#define M25 (B(1) | B(2) | B(3) | B(5) | B(6) | B(10) | B(11) | B(12) | B(13) | B(14) | B(17) | B(18) | B(20) | B(23))
#define M26 (B(2) | B(3) | B(4) | B(6) | B(7) | B(11) | B(12) | B(13) | B(14) | B(15) | B(18) | B(19) | B(21) | B(24))
#define M27 (B(1) | B(3) | B(4) | B(5) | B(7) | B(8) | B(12) | B(13) | B(14) | B(15) | B(16) | B(19) | B(20) | B(22))
#define M28 (B(2) | B(4) | B(5) | B(6) | B(8) | B(9) | B(13) | B(14) | B(15) | B(16) | B(17) | B(20) | B(21) | B(23))
#define M29 (B(1) | B(3) | B(5) | B(6) | B(7) | B(9) | B(10) | B(14) | B(15) | B(16) | B(17) | B(18) | B(21) | B(22) | B(24))
#define M30 (B(3) | B(5) | B(6) | B(8) | B(9) | B(10) | B(11) | B(13) | B(15) | B(19) | B(22) | B(23) | B(24))

#define PREAMBLE 0x8Bu

static int par(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (int)(x & 1u);
}

int lnav_parity(uint32_t w, int d29s, int d30s, uint32_t *data)
{
    uint32_t d = (w >> 6) & 0xFFFFFFu;
    if (d30s) {
        d ^= 0xFFFFFFu;
    }
    uint32_t p = ((uint32_t)(d29s ^ par(d & M25)) << 5) | ((uint32_t)(d30s ^ par(d & M26)) << 4) |
                 ((uint32_t)(d29s ^ par(d & M27)) << 3) | ((uint32_t)(d30s ^ par(d & M28)) << 2) |
                 ((uint32_t)(d30s ^ par(d & M29)) << 1) | (uint32_t)(d29s ^ par(d & M30));
    if (p != (w & 0x3Fu)) {
        return 0;
    }
    *data = d;
    return 1;
}

void lnav_init(lnav_t *l, int prn)
{
    memset(l, 0, sizeof(*l));
    l->prn = prn;
    l->week = -1;
}

/* Field of n bits starting at subframe bit a (1-based, as in IS-GPS-200), within one word's data. */
static uint32_t get(const uint32_t *dw, int a, int n)
{
    int w = (a - 1) / 30, pos = (a - 1) % 30;
    return (dw[w] >> (24 - pos - n)) & ((1u << n) - 1u);
}

static int32_t sext(uint32_t v, int n)
{
    uint32_t m = 1u << (n - 1);
    return (int32_t)((v ^ m) - m);
}

/* A 32-bit field split as 8 MSBs at a1 and 24 LSBs at a2. */
static uint32_t get32(const uint32_t *dw, int a1, int a2)
{
    return (get(dw, a1, 8) << 24) | get(dw, a2, 24);
}

static void decode_eph(lnav_t *l)
{
    const uint32_t *s1 = l->sf_data[0], *s2 = l->sf_data[1], *s3 = l->sf_data[2];
    int iodc = (int)((get(s1, 83, 2) << 8) | get(s1, 211, 8));
    int iode2 = (int)get(s2, 61, 8), iode3 = (int)get(s3, 271, 8);
    if (iode2 != iode3 || iode2 != (iodc & 0xFF)) {
        return;  /* a cut-over in progress: wait for a matching set */
    }
    gps_eph_t *e = &l->eph;
    const double P = GPS_PI_ICD;
    e->prn = l->prn;
    e->week = (int)get(s1, 61, 10) + 2048;  /* second rollover era: 2019-04-07 to 2038 */
    e->ura = (int)get(s1, 73, 4);
    e->health = (int)get(s1, 77, 6);
    e->iodc = iodc;
    e->iode = iode2;
    e->tgd = sext(get(s1, 197, 8), 8) * 0x1p-31;
    e->toc = get(s1, 219, 16) * 16.0;
    e->af2 = sext(get(s1, 241, 8), 8) * 0x1p-55;
    e->af1 = sext(get(s1, 249, 16), 16) * 0x1p-43;
    e->af0 = sext(get(s1, 271, 22), 22) * 0x1p-31;

    e->crs = sext(get(s2, 69, 16), 16) * 0x1p-5;
    e->delta_n = sext(get(s2, 91, 16), 16) * 0x1p-43 * P;
    e->m0 = (int32_t)get32(s2, 107, 121) * 0x1p-31 * P;
    e->cuc = sext(get(s2, 151, 16), 16) * 0x1p-29;
    e->e = get32(s2, 167, 181) * 0x1p-33;
    e->cus = sext(get(s2, 211, 16), 16) * 0x1p-29;
    e->sqrt_a = get32(s2, 227, 241) * 0x1p-19;
    e->toe = get(s2, 271, 16) * 16.0;

    e->cic = sext(get(s3, 61, 16), 16) * 0x1p-29;
    e->omega0 = (int32_t)get32(s3, 77, 91) * 0x1p-31 * P;
    e->cis = sext(get(s3, 121, 16), 16) * 0x1p-29;
    e->i0 = (int32_t)get32(s3, 137, 151) * 0x1p-31 * P;
    e->crc = sext(get(s3, 181, 16), 16) * 0x1p-5;
    e->omega = (int32_t)get32(s3, 197, 211) * 0x1p-31 * P;
    e->omega_dot = sext(get(s3, 241, 24), 24) * 0x1p-43 * P;
    e->idot = sext(get(s3, 279, 14), 14) * 0x1p-43 * P;
    e->valid = 1;
}

static void decode_iono(lnav_t *l, const uint32_t *dw)
{
    gps_iono_t *io = &l->iono;
    io->alpha[0] = sext(get(dw, 69, 8), 8) * 0x1p-30;
    io->alpha[1] = sext(get(dw, 77, 8), 8) * 0x1p-27;
    io->alpha[2] = sext(get(dw, 91, 8), 8) * 0x1p-24;
    io->alpha[3] = sext(get(dw, 99, 8), 8) * 0x1p-24;
    io->beta[0] = sext(get(dw, 107, 8), 8) * 0x1p11;
    io->beta[1] = sext(get(dw, 121, 8), 8) * 0x1p14;
    io->beta[2] = sext(get(dw, 129, 8), 8) * 0x1p16;
    io->beta[3] = sext(get(dw, 137, 8), 8) * 0x1p16;
    io->valid = 1;
}

/* Tries the 302 buffered bits as [D29*, D30*, subframe]. Returns the subframe ID, or 0. */
static int try_subframe(lnav_t *l)
{
    uint8_t b[LNAV_SF_BITS + 2];
    uint32_t pre = 0;
    for (int k = 0; k < 8; k++) {
        pre = (pre << 1) | l->bits[2 + k];
    }
    int inv;
    if (pre == PREAMBLE) {
        inv = 0;
    } else if (pre == (~PREAMBLE & 0xFFu)) {
        inv = 1;
    } else {
        return 0;
    }
    for (int k = 0; k < LNAV_SF_BITS + 2; k++) {
        b[k] = (uint8_t)(l->bits[k] ^ inv);
    }
    uint32_t dw[10];
    int d29 = b[0], d30 = b[1];
    for (int w = 0; w < 10; w++) {
        uint32_t word = 0;
        for (int k = 0; k < 30; k++) {
            word = (word << 1) | b[2 + 30 * w + k];
        }
        if (!lnav_parity(word, d29, d30, &dw[w])) {
            l->n_parity_fail++;
            return 0;
        }
        d29 = (int)((word >> 1) & 1u);
        d30 = (int)(word & 1u);
    }
    int id = (int)((dw[1] >> 2) & 7u);
    uint32_t tow_count = dw[1] >> 7;
    if (id < 1 || id > 5) {
        return 0;
    }
    l->synced = 1;
    l->inverted = inv;
    l->sf_period = l->period[2];
    l->sf_tow = (double)tow_count * 6.0 - 6.0;  /* HOW gives the start of the next subframe */
    if (l->sf_tow < 0.0) {
        l->sf_tow += 604800.0;
    }
    l->n_subframes++;
    if (id <= 3) {
        memcpy(l->sf_data[id - 1], dw, sizeof(dw));
        l->have_sf |= 1 << (id - 1);
        if (l->have_sf == 7) {
            decode_eph(l);
            l->have_sf = 0;
        }
        if (id == 1) {
            l->week = (int)get(dw, 61, 10) + 2048;
        }
    } else if (id == 4 && get(dw, 63, 6) == 56) {
        decode_iono(l, dw);
    }
    return id;
}

int lnav_push(lnav_t *l, int bit, uint32_t period)
{
    const int cap = LNAV_SF_BITS + 2;
    if (l->nbits == cap) {
        memmove(l->bits, l->bits + 1, (size_t)(cap - 1));
        memmove(l->period, l->period + 1, sizeof(uint32_t) * (size_t)(cap - 1));
        l->nbits--;
    }
    l->bits[l->nbits] = (uint8_t)(bit > 0);
    l->period[l->nbits] = period;
    l->nbits++;
    if (l->nbits < cap) {
        return 0;
    }
    int id = try_subframe(l);
    if (id) {
        /* Keep the last two bits: they are D29* and D30* for the next subframe. */
        memmove(l->bits, l->bits + LNAV_SF_BITS, 2);
        memmove(l->period, l->period + LNAV_SF_BITS, sizeof(uint32_t) * 2);
        l->nbits = 2;
    }
    return id;
}
