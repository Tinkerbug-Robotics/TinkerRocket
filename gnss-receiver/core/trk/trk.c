#include "gnss/trk.h"

#include "gnss/gmath.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F 6.2831853071795864769f
#define CHIP_PER_HZ 6.4935064935e-4f   /* 1.023e6 / 1575.42e6: code Doppler per carrier Doppler */

const trk_profile_t trk_profile_quiet = {{10.0f, 15.0f, 2.0f}, {0.0f, 10.0f, 0.25f}, 2};
const trk_profile_t trk_profile_boost = {{10.0f, 50.0f, 2.0f}, {5.0f, 50.0f, 1.0f}, 2};

static float bw_step(float cur, float target, float k)
{
    if (target >= cur) {
        return target;
    }
    float v = cur + k * (target - cur);
    return v < target + 0.01f ? target : v;
}

void trk_profile_step(trk_profile_t *cur, const trk_profile_t *target, float dt)
{
    float k = dt / TRK_NARROW_TAU;
    if (k > 1.0f) {
        k = 1.0f;
    }
    const trk_bw_t *t[2] = {&target->pullin, &target->locked};
    trk_bw_t *c[2] = {&cur->pullin, &cur->locked};
    for (int i = 0; i < 2; i++) {
        c[i]->fll_bw = bw_step(c[i]->fll_bw, t[i]->fll_bw, k);
        c[i]->pll_bw = bw_step(c[i]->pll_bw, t[i]->pll_bw, k);
        c[i]->dll_bw = bw_step(c[i]->dll_bw, t[i]->dll_bw, k);
    }
    cur->fll_ms = target->fll_ms;
}

/* PULLIN ends when the lock indicator has stayed above this; LOCKED falls back below the other. */
#define LOCK_IN   0.85f
#define LOCK_OUT  0.4f
#define LOCK_TAU  0.1f   /* lock indicator time constant, s */
#define MIN_PULLIN_S 0.3f
/* Bit sync: transitions needed, and the share the winning bin must hold. */
#define SYNC_TRANS 30
#define SYNC_SHARE 0.6f
/* The narrowband/wideband C/N0 estimate: blocks of NW_M dumps inside a bit (5 ms loses under
 * 1 dB to a 50 Hz frequency error, where a whole 20 ms bit would lose everything), averaged
 * over NW_K blocks (200 ms). */
#define NW_M 5u
#define NW_K 40
/* No line of sight changes its Doppler faster: 3 kHz/s is 58 g along it. The loops' rate state
 * stays inside this, so a loop that has lost its signal cannot run the NCO away. */
#define MAX_RATE_RAD_S2 (TWO_PI_F * 3000.0f)
/* A loop whose signal has gone coasts: frequency held, rate decaying with this time constant. */
#define COAST_TAU 0.5f

void trk_start(trk_ch_t *c, int prn, float dop_hz, float tap_chips, int32_t if_word, uint64_t code_word0,
               float carr_k, float code_k)
{
    memset(c, 0, sizeof(*c));
    c->state = TRK_PULLIN;
    c->prn = (uint8_t)prn;
    c->tap_chips = tap_chips;
    c->if_word = if_word;
    c->code_word0 = code_word0;
    c->carr_k = carr_k;
    c->code_k = code_k;
    c->dop_hz = dop_hz;
    c->x2 = TWO_PI_F * dop_hz;
}

static void enter(trk_ch_t *c, trk_state_t s)
{
    c->state = s;
    c->t_state = 0.0f;
}

/*
 * The FLL's frequency error (rad/s) and the time it stands for (s), from consecutive blocks of
 * fll_ms dumps. Once bits are synchronized the blocks sit inside a data bit and the
 * discriminator is a full atan2 (+-1 / (2 fll_ms ms)); pairs that straddle a bit edge give
 * nothing. Before that, the blocks run unaligned and the discriminator is folded for data bits
 * (+-1 / (4 fll_ms ms)). Returns 0 time when this dump completes no measurement.
 */
static float fll_error(trk_ch_t *c, int fll_ms, uint32_t p, float ip, float qp, float T, float *t_meas)
{
    *t_meas = 0.0f;
    const uint32_t n = fll_ms > 1 ? (uint32_t)fll_ms : 1u;
    const int aligned = c->bit_sync && n > 1;
    if (aligned != c->blk_aligned) {
        c->blk_aligned = aligned;
        c->blk_have_prev = 0;
        c->blk_n = 0;
    }
    uint32_t pos = aligned ? (p + 20u - c->bit_phase) % 20u : p;  /* place in the bit, or just a count */
    if (pos % n == 0) {
        if (aligned && pos == 0) {
            c->blk_have_prev = 0;  /* a new bit: no block before this one shares its sign */
        }
        c->blk_i = c->blk_q = 0.0f;
        c->blk_n = 0;
    }
    c->blk_i += ip;
    c->blk_q += qp;
    c->blk_n++;
    if (pos % n != n - 1) {
        return 0.0f;
    }
    float ef = 0.0f;
    const float tb = T * (float)n;
    if (c->blk_have_prev && c->blk_n == n) {
        float cross = c->blk_prev_i * c->blk_q - c->blk_prev_q * c->blk_i;
        float dot = c->blk_prev_i * c->blk_i + c->blk_prev_q * c->blk_q;
        ef = (aligned ? gnss_atan2f(cross, dot) : gnss_atan_halff(cross, dot)) / tb;
        *t_meas = tb;
    }
    c->blk_prev_i = c->blk_i;
    c->blk_prev_q = c->blk_q;
    c->blk_have_prev = c->blk_n == n;
    return ef;
}

static void carrier_loop(trk_ch_t *c, const trk_bw_t *bw, int fll_ms, uint32_t p, float ip, float qp, float T)
{
    float ep = gnss_atan_halff(qp, ip);  /* Costas: data-insensitive, rad */
    float ef = 0.0f, tf = 0.0f;
    if (bw->fll_bw > 0.0f) {
        ef = fll_error(c, fll_ms, p, ip, qp, T, &tf);
    }
    float wp = bw->pll_bw / 0.7845f;
    float wf = bw->fll_bw / 0.53f;
    /* The FLL's measurement enters once per block it spans, weighted by that block's time. */
    c->x1 += T * wp * wp * wp * ep + tf * wf * wf * ef;
    if (c->x1 > MAX_RATE_RAD_S2) {
        c->x1 = MAX_RATE_RAD_S2;
    } else if (c->x1 < -MAX_RATE_RAD_S2) {
        c->x1 = -MAX_RATE_RAD_S2;
    }
    c->x2 += T * (c->x1 + TWO_PI_F * c->ff_rate + 1.1f * wp * wp * ep) + tf * 1.414f * wf * ef;
    c->dop_hz = (c->x2 + 2.4f * wp * ep) / TWO_PI_F;
}

/* No signal to steer by: hold the frequency, let the rate decay, and stop the DLL. */
static void coast(trk_ch_t *c, float T)
{
    c->x1 -= (T / COAST_TAU) * c->x1;
    c->x2 += T * (c->x1 + TWO_PI_F * c->ff_rate);
    c->dop_hz = c->x2 / TWO_PI_F;
    c->dll_rate = 0.0f;
    c->blk_have_prev = 0;
    c->have_prev = 0;
}

static void code_loop(trk_ch_t *c, const trk_bw_t *bw, const corr_dump_t *d)
{
    float e = sqrtf(d->ie * d->ie + d->qe * d->qe);
    float l = sqrtf(d->il * d->il + d->ql * d->ql);
    float s = e + l;
    /* Normalized envelope: error in chips for a triangle, spacing 2 * tap. */
    float err = s > 0.0f ? (e - l) / s * (1.0f - c->tap_chips) : 0.0f;
    c->dll_rate = 4.0f * bw->dll_bw * err;
}

/* Feeds one C/N0 estimate (linear, Hz) to the loss timer, which counts time below 25 dB-Hz. */
static void cn0_estimate(trk_ch_t *c, float snr, float span)
{
    c->cn0 = snr > 0.0f ? 10.0f * log10f(snr) : 0.0f;
    /* Thresholds compare in the linear domain, so no libm result decides anything. */
    if (snr < 316.22777f) {  /* 25 dB-Hz */
        c->t_weak += span;
    } else {
        c->t_weak = 0.0f;
    }
}

static void lock_and_cn0(trk_ch_t *c, uint32_t p, float ip, float qp, float T)
{
    float p2 = ip * ip + qp * qp;
    /* Lock: cos 2(phase error), as low-passed I^2 - Q^2 over low-passed signal power. Taking the
     * noise power out of the denominator makes it read 1 in lock at any C/N0; per-dump
     * (I^2 - Q^2) / (I^2 + Q^2) reads SNR / (SNR + 1), which never reaches LOCK_IN below 38 dB-Hz. */
    float k = T / LOCK_TAU;
    c->nbd += k * ((ip * ip - qp * qp) - c->nbd);
    c->nbp += k * (p2 - c->nbp);
    float pd_lp = c->nbp - c->pn;
    c->pll_lock = pd_lp > 0.0f ? c->nbd / pd_lp : 0.0f;

    if (c->bit_sync) {
        /*
         * Narrowband over wideband power (Van Dierendonck), in blocks of M dumps inside a bit:
         * mean(NP / WP) = mu gives the SNR per dump (mu - 1) / (M - mu). Unlike the moments
         * estimate below, it reads near zero on noise alone, so a lost signal is seen as lost.
         */
        uint32_t pos = (p + 20u - c->bit_phase) % 20u;
        if (pos % NW_M == 0) {
            c->nw_i = c->nw_q = c->nw_wp = 0.0f;
            c->nw_n = 0;
        }
        c->nw_i += ip;
        c->nw_q += qp;
        c->nw_wp += p2;
        c->nw_n++;
        if (pos % NW_M == NW_M - 1 && c->nw_n == NW_M && c->nw_wp > 0.0f) {
            c->nw_mu += (c->nw_i * c->nw_i + c->nw_q * c->nw_q) / c->nw_wp;
            c->nw_wsum += c->nw_wp;
            if (++c->nw_k == NW_K) {
                const float m = (float)NW_M;
                float mu = c->nw_mu / NW_K;
                float s = mu > 1.0f && mu < m ? (mu - 1.0f) / (m - mu) : 0.0f;
                float pt = c->nw_wsum / (m * NW_K);  /* signal + noise power per dump */
                c->pn = pt / (1.0f + s);
                cn0_estimate(c, s / T, T * m * NW_K);
                c->nw_mu = c->nw_wsum = 0.0f;
                c->nw_k = 0;
            }
        }
        c->m2 = c->m4 = 0.0f;
        c->nm = 0;
        return;
    }
    c->nw_k = 0;
    c->nw_mu = c->nw_wsum = 0.0f;
    c->m2 += p2;
    c->m4 += p2 * p2;
    if (++c->nm == TRK_CN0_N) {
        float m2 = c->m2 / TRK_CN0_N, m4 = c->m4 / TRK_CN0_N;
        float pd2 = 2.0f * m2 * m2 - m4;
        float pd = pd2 > 0.0f ? sqrtf(pd2) : 0.0f;
        float pn = m2 - pd;
        if (pn > 0.0f) {
            c->pn = pn;
        }
        cn0_estimate(c, (pn > 0.0f) ? pd / (pn * T) : 0.0f, T * TRK_CN0_N);
        c->m2 = c->m4 = 0.0f;
        c->nm = 0;
    }
}

static int bits(trk_ch_t *c, uint32_t p, float ip, int *bit, uint32_t *bit_period)
{
    float sign = ip >= 0.0f ? 1.0f : -1.0f;
    if (!c->bit_sync) {
        if (c->state == TRK_LOCKED && c->t_state > 0.1f) {
            if (c->prev_sign != 0.0f && sign != c->prev_sign) {
                c->hist[p % 20]++;
                c->n_trans++;
            }
            if (c->n_trans >= SYNC_TRANS) {
                int b = 0;
                for (int k = 1; k < 20; k++) {
                    if (c->hist[k] > c->hist[b]) {
                        b = k;
                    }
                }
                if (c->hist[b] >= SYNC_SHARE * c->n_trans) {
                    c->bit_sync = 1;
                    c->bit_phase = (uint8_t)b;
                    c->bit_n = 0;
                } else if (c->n_trans > 10 * SYNC_TRANS) {
                    memset(c->hist, 0, sizeof(c->hist));
                    c->n_trans = 0;
                }
            }
        }
        c->prev_sign = sign;
        return 0;
    }
    c->prev_sign = sign;
    if (p % 20 == c->bit_phase) {
        c->bit_sum = ip;
        c->bit_n = 1;
        c->bit_first = p;
    } else if (c->bit_n > 0) {
        c->bit_sum += ip;
        c->bit_n++;
    }
    if (c->bit_n == 20) {
        c->bit_n = 0;
        *bit = c->bit_sum >= 0.0f ? 1 : -1;
        *bit_period = c->bit_first;
        return 1;
    }
    return 0;
}

int trk_update(trk_ch_t *c, const trk_profile_t *p, const corr_dump_t *d, float T, int *bit, uint32_t *bit_period)
{
    if (c->state == TRK_OFF) {
        return 0;
    }
    const trk_bw_t *bw = (c->state == TRK_LOCKED) ? &p->locked : &p->pullin;
    c->period = d->seq;
    if (c->t_weak > 0.0f) {
        coast(c, T);  /* the last C/N0 estimate saw no signal */
    } else {
        carrier_loop(c, bw, p->fll_ms, d->seq, d->ip, d->qp, T);
        code_loop(c, bw, d);
    }
    lock_and_cn0(c, d->seq, d->ip, d->qp, T);
    c->prev_ip = d->ip;
    c->prev_qp = d->qp;
    c->have_prev = 1;
    c->t_state += T;

    if (c->state == TRK_PULLIN && c->t_state > MIN_PULLIN_S && c->pll_lock > LOCK_IN) {
        enter(c, TRK_LOCKED);
    } else if (c->state == TRK_LOCKED && c->pll_lock < LOCK_OUT) {
        /* Bit sync stays: bit edges follow the code, which the FLL and DLL still hold. */
        enter(c, TRK_PULLIN);
    }
    if (c->t_weak >= TRK_LOSS_S) {
        enter(c, TRK_OFF);
        return 0;
    }
    return bits(c, d->seq, d->ip, bit, bit_period);
}

void trk_words(const trk_ch_t *c, int32_t *carr_word, uint64_t *code_word)
{
    *carr_word = c->if_word + (int32_t)lrintf(c->dop_hz * c->carr_k);
    float rate = c->dop_hz * CHIP_PER_HZ + c->dll_rate;  /* chips/s beyond nominal */
    *code_word = c->code_word0 + (uint64_t)(int64_t)lrintf(rate * c->code_k);
}
