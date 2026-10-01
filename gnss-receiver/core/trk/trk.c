#include "gnss/trk.h"

#include "gnss/gmath.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F 6.2831853071795864769f
#define CHIP_PER_HZ 6.4935064935e-4f   /* 1.023e6 / 1575.42e6: code Doppler per carrier Doppler */

static const trk_bw_t kPullin = {10.0f, 15.0f, 2.0f};
static const trk_bw_t kLocked = {0.0f, 10.0f, 0.25f};

/* PULLIN ends when the lock indicator has stayed above this; LOCKED falls back below the other. */
#define LOCK_IN   0.85f
#define LOCK_OUT  0.4f
#define LOCK_TAU  0.1f   /* lock indicator time constant, s */
#define MIN_PULLIN_S 0.3f
/* Bit sync: transitions needed, and the share the winning bin must hold. */
#define SYNC_TRANS 30
#define SYNC_SHARE 0.6f

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

static void carrier_loop(trk_ch_t *c, const trk_bw_t *bw, float ip, float qp, float T)
{
    float ep = gnss_atan_halff(qp, ip);  /* Costas: data-insensitive, rad */
    float ef = 0.0f;
    if (c->have_prev && bw->fll_bw > 0.0f) {
        float cross = c->prev_ip * qp - c->prev_qp * ip;
        float dot = c->prev_ip * ip + c->prev_qp * qp;
        ef = gnss_atan_halff(cross, dot) / T;  /* rad/s, folded for data bits */
    }
    float wp = bw->pll_bw / 0.7845f;
    float wf = bw->fll_bw / 0.53f;
    c->x1 += T * (wp * wp * wp * ep + wf * wf * ef);
    c->x2 += T * (c->x1 + 1.1f * wp * wp * ep + 1.414f * wf * ef);
    c->dop_hz = (c->x2 + 2.4f * wp * ep) / TWO_PI_F;
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

static void lock_and_cn0(trk_ch_t *c, float ip, float qp, float T)
{
    float p2 = ip * ip + qp * qp;
    if (p2 > 0.0f) {
        float c2 = (ip * ip - qp * qp) / p2;
        c->pll_lock += (T / LOCK_TAU) * (c2 - c->pll_lock);
    }
    c->m2 += p2;
    c->m4 += p2 * p2;
    if (++c->nm == TRK_CN0_N) {
        float m2 = c->m2 / TRK_CN0_N, m4 = c->m4 / TRK_CN0_N;
        float pd2 = 2.0f * m2 * m2 - m4;
        float pd = pd2 > 0.0f ? sqrtf(pd2) : 0.0f;
        float pn = m2 - pd;
        float snr = (pn > 0.0f) ? pd / (pn * T) : 0.0f;  /* C/N0, linear */
        c->cn0 = snr > 0.0f ? 10.0f * log10f(snr) : 0.0f;
        c->m2 = c->m4 = 0.0f;
        c->nm = 0;
        /* Thresholds compare in the linear domain, so no libm result decides anything. */
        if (snr < 316.22777f) {  /* 25 dB-Hz */
            c->t_weak += T * TRK_CN0_N;
        } else {
            c->t_weak = 0.0f;
        }
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

int trk_update(trk_ch_t *c, const corr_dump_t *d, float T, int *bit, uint32_t *bit_period)
{
    if (c->state == TRK_OFF) {
        return 0;
    }
    const trk_bw_t *bw = (c->state == TRK_LOCKED) ? &kLocked : &kPullin;
    c->period = d->seq;
    carrier_loop(c, bw, d->ip, d->qp, T);
    code_loop(c, bw, d);
    lock_and_cn0(c, d->ip, d->qp, T);
    c->prev_ip = d->ip;
    c->prev_qp = d->qp;
    c->have_prev = 1;
    c->t_state += T;

    if (c->state == TRK_PULLIN && c->t_state > MIN_PULLIN_S && c->pll_lock > LOCK_IN) {
        enter(c, TRK_LOCKED);
    } else if (c->state == TRK_LOCKED && c->pll_lock < LOCK_OUT) {
        enter(c, TRK_PULLIN);
        c->bit_sync = 0;
        memset(c->hist, 0, sizeof(c->hist));
        c->n_trans = 0;
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
