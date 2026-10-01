#include "corr_model.h"

#include "fe_format.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.283185307179586476925286766559

void corr_model_default_cfg(corr_model_cfg_t *c)
{
    static const int8_t cs[] = CORR_CARR_COS_LUT;
    static const int8_t sn[] = CORR_CARR_SIN_LUT;
    memset(c, 0, sizeof(*c));
    c->lut_bits = CORR_CARR_LUT_BITS;
    memcpy(c->cos_lut, cs, sizeof(cs));
    memcpy(c->sin_lut, sn, sizeof(sn));
    c->acc_bits = CORR_ACC_BITS;
}

int corr_model_lut(corr_model_cfg_t *c, int bits, double amp)
{
    if (bits < 1 || bits > CM_MAX_LUT_BITS) {
        return -1;
    }
    c->lut_bits = bits;
    int n = 1 << bits;
    for (int k = 0; k < n; k++) {
        double a = TWO_PI * ((double)k + 0.5) / (double)n;
        c->cos_lut[k] = (int8_t)lround(amp * cos(a));
        c->sin_lut[k] = (int8_t)lround(amp * sin(a));
    }
    return 0;
}

static int weight(unsigned sign, unsigned mag)
{
    int w = mag ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL;
    return sign ? -w : w;
}

void corr_model_init(corr_model_t *m, const corr_model_cfg_t *cfg)
{
    memset(m, 0, sizeof(*m));
    m->cfg = *cfg;
    for (int prn = 1; prn <= GPS_MAX_PRN; prn++) {
        gps_ca_code(prn, m->ca[prn - 1]);
    }
    int ns = 1 << cfg->lut_bits;
    for (unsigned nib = 0; nib < 16; nib++) {
        int wi = weight(nib & FE_CODE_I_SIGN, nib & FE_CODE_I_MAG);
        int wq = weight(nib & FE_CODE_Q_SIGN, nib & FE_CODE_Q_MAG);
        for (int k = 0; k < ns; k++) {
            int c = cfg->cos_lut[k], s = cfg->sin_lut[k];
            m->mix_i[nib][k] = (int8_t)(wi * c + wq * s);
            m->mix_q[nib][k] = (int8_t)(wq * c - wi * s);
        }
    }
}

int corr_model_command(corr_model_t *m, const corr_cmd_t *cmd)
{
    if (cmd->ch >= CORR_MAX_CH) {
        return -1;
    }
    cm_ch_t *h = &m->ch[cmd->ch];
    switch (cmd->type) {
    case CORR_CMD_START:
        if (cmd->sig != GNSS_SIG_GPS_L1CA || cmd->prn < 1 || cmd->prn > GPS_MAX_PRN) {
            return -1;
        }
        h->start = *cmd;
        h->start_pending = 1;
        return 0;
    case CORR_CMD_NCO:
        h->carr_word_next = cmd->carr_word;
        h->code_word_next = cmd->code_word;
        h->nco_pending = 1;
        return 0;
    case CORR_CMD_STOP:
        h->active = 0;
        h->start_pending = 0;
        h->nco_pending = 0;
        return 0;
    default:
        return -1;
    }
}

static void begin(corr_model_t *m, cm_ch_t *h)
{
    const corr_cmd_t *s = &h->start;
    h->code = m->ca[s->prn - 1];
    h->code_mod = (uint64_t)GPS_CA_LEN << CORR_CODE_FRAC_BITS;
    h->code_phase = s->code_phase % h->code_mod;
    h->code_word = s->code_word;
    h->tap = s->tap_offset;
    h->carr_word = s->carr_word;
    h->carr_phase = 0;
    h->carr_cycles = 0;
    h->nco_pending = 0;
    memset(h->acc, 0, sizeof(h->acc));
    h->seq = 0;
    h->active = 1;
    h->start_pending = 0;
}

static void check_acc(corr_model_t *m, const int32_t *acc)
{
    const int32_t lim = (int32_t)1 << (m->cfg.acc_bits - 1);
    int over = 0;
    for (int k = 0; k < 6; k++) {
        int32_t a = acc[k] < 0 ? -acc[k] : acc[k];
        if (a > m->acc_peak) {
            m->acc_peak = a;
        }
        over |= (acc[k] >= lim || acc[k] < -lim);
    }
    m->acc_overflows += (uint32_t)over;
}

static int run(corr_model_t *m, cm_ch_t *h, uint64_t t0, const uint8_t *codes, size_t k0, size_t n,
               corr_dump_t *dumps, int max, uint8_t ch)
{
    const int shift = 32 - m->cfg.lut_bits;
    const uint8_t *code = h->code;
    const uint64_t mod = h->code_mod, tap = h->tap;
    uint32_t ph = h->carr_phase, cyc = h->carr_cycles;
    uint64_t cp = h->code_phase;
    int32_t ie = h->acc[0], qe = h->acc[1], ip = h->acc[2], qp = h->acc[3], il = h->acc[4], ql = h->acc[5];
    int nd = 0;
    for (size_t k = k0; k < n; k++) {
        unsigned nib = codes[k] & 0xFu, sec = ph >> shift;
        int32_t mi = m->mix_i[nib][sec], mq = m->mix_q[nib][sec];
        uint64_t ce = cp + tap;
        if (ce >= mod) {
            ce -= mod;
        }
        uint64_t cl = (cp >= tap) ? cp - tap : cp + mod - tap;
        if (code[ce >> CORR_CODE_FRAC_BITS]) {
            ie -= mi;
            qe -= mq;
        } else {
            ie += mi;
            qe += mq;
        }
        if (code[cp >> CORR_CODE_FRAC_BITS]) {
            ip -= mi;
            qp -= mq;
        } else {
            ip += mi;
            qp += mq;
        }
        if (code[cl >> CORR_CODE_FRAC_BITS]) {
            il -= mi;
            ql -= mq;
        } else {
            il += mi;
            ql += mq;
        }
        uint32_t old = ph;
        ph += (uint32_t)h->carr_word;
        if (h->carr_word >= 0) {
            cyc += (ph < old);
        } else {
            cyc -= (ph > old);
        }
        cp += h->code_word;
        if (cp >= mod) {
            cp -= mod;
            int32_t acc[6] = {ie, qe, ip, qp, il, ql};
            check_acc(m, acc);
            if (nd < max) {
                corr_dump_t *d = &dumps[nd++];
                d->ch = ch;
                d->seq = h->seq;
                d->t_samp = t0 + k + 1;
                d->code_phase = cp;
                d->carr_phase = ph;
                d->carr_cycles = cyc;
                d->carr_word = h->carr_word;
                d->code_word = h->code_word;
                d->ie = (float)ie;
                d->qe = (float)qe;
                d->ip = (float)ip;
                d->qp = (float)qp;
                d->il = (float)il;
                d->ql = (float)ql;
            }
            h->seq++;
            ie = qe = ip = qp = il = ql = 0;
            if (h->nco_pending) {
                h->carr_word = h->carr_word_next;
                h->code_word = h->code_word_next;
                h->nco_pending = 0;
            }
        }
    }
    h->carr_phase = ph;
    h->carr_cycles = cyc;
    h->code_phase = cp;
    h->acc[0] = ie;
    h->acc[1] = qe;
    h->acc[2] = ip;
    h->acc[3] = qp;
    h->acc[4] = il;
    h->acc[5] = ql;
    return nd;
}

int corr_model_process(corr_model_t *m, uint64_t t0, const uint8_t *codes, size_t n, corr_dump_t *dumps, int max)
{
    int nd = 0;
    for (int ch = 0; ch < CORR_MAX_CH; ch++) {
        cm_ch_t *h = &m->ch[ch];
        size_t k0 = 0;
        if (h->start_pending) {
            if (h->start.t_start >= t0 + n) {
                continue;
            }
            k0 = h->start.t_start > t0 ? (size_t)(h->start.t_start - t0) : 0;
            begin(m, h);
        }
        if (!h->active) {
            continue;
        }
        nd += run(m, h, t0, codes, k0, n, dumps + nd, max - nd, (uint8_t)ch);
    }
    return nd;
}
