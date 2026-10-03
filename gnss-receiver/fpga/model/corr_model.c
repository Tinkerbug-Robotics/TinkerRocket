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
        if (!corr_model_sig_ok(cmd->sig, cmd->prn)) {
            return -1;
        }
        h->start = *cmd;
        h->start_pending = 1;
        return 0;
    case CORR_CMD_NCO:
        cmdq_push(&h->q, cmd->apply_seq, cmd->carr_word, cmd->code_word);
        return 0;
    case CORR_CMD_STOP:
        h->active = 0;
        h->start_pending = 0;
        cmdq_clear(&h->q);
        return 0;
    default:
        return -1;
    }
}

int corr_model_sig_ok(int sig, int prn)
{
    switch (sig) {
    case GNSS_SIG_GPS_L1CA:
        return prn >= 1 && prn <= GPS_MAX_PRN;
    case GNSS_SIG_GAL_E1C:
        return prn >= 1 && prn <= GAL_MAX_PRN;
    case GNSS_SIG_BDS_B1CP:
        return prn >= 1 && prn <= BDS_MAX_PRN;
    default:
        return 0;
    }
}

/* The channel's codes for its signal (the HDL reads them from block RAM or generates them). */
static int load_codes(cm_ch_t *h, int sig, int prn)
{
    switch (sig) {
    case GNSS_SIG_GPS_L1CA:
        gps_ca_code(prn, h->code);
        h->has_data = 0;
        h->boc = 0;
        return GPS_CA_LEN;
    case GNSS_SIG_GAL_E1C:
        gal_e1_code(prn, GNSS_SIG_GAL_E1C, h->code);
        gal_e1_code(prn, GNSS_SIG_GAL_E1B, h->dcode);
        h->has_data = 1;
        h->boc = 1;
        return GAL_E1_LEN;
    default:  /* GNSS_SIG_BDS_B1CP: corr_model_sig_ok checked it */
        bds_b1c_code(prn, GNSS_SIG_BDS_B1CP, h->code);
        bds_b1c_code(prn, GNSS_SIG_BDS_B1CD, h->dcode);
        h->has_data = 1;
        h->boc = 1;
        return BDS_B1C_LEN;
    }
}

static void begin(corr_model_t *m, cm_ch_t *h)
{
    (void)m;
    const corr_cmd_t *s = &h->start;
    h->code_mod = (uint64_t)load_codes(h, s->sig, s->prn) << CORR_CODE_FRAC_BITS;
    h->code_phase = s->code_phase % h->code_mod;
    h->code_word = s->code_word;
    h->tap = s->tap_offset;
    h->tap2 = s->tap_offset2;
    h->carr_word = s->carr_word;
    h->carr_phase = 0;
    h->carr_cycles = 0;
    cmdq_clear(&h->q);
    memset(h->acc, 0, sizeof(h->acc));
    h->seq = 0;
    h->active = 1;
    h->start_pending = 0;
}

static void check_acc(corr_model_t *m, const int32_t *acc)
{
    const int32_t lim = (int32_t)1 << (m->cfg.acc_bits - 1);
    int over = 0;
    for (int k = 0; k < 2 * CM_NTAPS; k++) {
        int32_t a = acc[k] < 0 ? -acc[k] : acc[k];
        if (a > m->acc_peak) {
            m->acc_peak = a;
        }
        over |= (acc[k] >= lim || acc[k] < -lim);
    }
    m->acc_overflows += (uint32_t)over;
}

/* The replica chip at code phase x: the code, flipped in the second half of each chip for BOC(1,1). */
static inline unsigned chip_at(const uint8_t *code, uint64_t x, int boc)
{
    unsigned c = code[x >> CORR_CODE_FRAC_BITS];
    return boc ? c ^ (unsigned)((x >> (CORR_CODE_FRAC_BITS - 1)) & 1u) : c;
}

static inline void acc_add(int32_t *a, unsigned chip, int32_t mi, int32_t mq)
{
    if (chip) {
        a[0] -= mi;
        a[1] -= mq;
    } else {
        a[0] += mi;
        a[1] += mq;
    }
}

static int run(corr_model_t *m, cm_ch_t *h, uint64_t t0, const uint8_t *codes, size_t k0, size_t n,
               corr_dump_t *dumps, int max, uint8_t ch)
{
    const int shift = 32 - m->cfg.lut_bits;
    const uint8_t *code = h->code, *dcode = h->dcode;
    const uint64_t mod = h->code_mod, tap = h->tap, tap2 = h->tap2;
    const int boc = h->boc, has_data = h->has_data;
    uint32_t ph = h->carr_phase, cyc = h->carr_cycles;
    uint64_t cp = h->code_phase;
    int32_t acc[2 * CM_NTAPS];
    memcpy(acc, h->acc, sizeof(acc));
    int nd = 0;
    for (size_t k = k0; k < n; k++) {
        unsigned nib = codes[k] & 0xFu, sec = ph >> shift;
        int32_t mi = m->mix_i[nib][sec], mq = m->mix_q[nib][sec];
        uint64_t xe = cp + tap, xve = cp + tap2;
        if (xe >= mod) {
            xe -= mod;
        }
        if (xve >= mod) {
            xve -= mod;
        }
        uint64_t xl = (cp >= tap) ? cp - tap : cp + mod - tap;
        uint64_t xvl = (cp >= tap2) ? cp - tap2 : cp + mod - tap2;
        acc_add(&acc[2 * CM_E], chip_at(code, xe, boc), mi, mq);
        acc_add(&acc[2 * CM_P], chip_at(code, cp, boc), mi, mq);
        acc_add(&acc[2 * CM_L], chip_at(code, xl, boc), mi, mq);
        acc_add(&acc[2 * CM_VE], chip_at(code, xve, boc), mi, mq);
        acc_add(&acc[2 * CM_VL], chip_at(code, xvl, boc), mi, mq);
        if (has_data) {
            acc_add(&acc[2 * CM_D], chip_at(dcode, cp, boc), mi, mq);
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
            check_acc(m, acc);
            int32_t next_carr = h->carr_word;
            uint64_t next_code = h->code_word;
            uint8_t flags = cmdq_epoch(&h->q, h->seq, &next_carr, &next_code);
            if (nd < max) {
                corr_dump_t *d = &dumps[nd++];
                d->flags = flags;
                d->ch = ch;
                d->seq = h->seq;
                d->t_samp = (t0 + k + 1) & CORR_TSAMP_MASK;
                d->code_phase = cp;
                d->carr_phase = ph;
                d->carr_cycles = cyc;
                d->carr_word = h->carr_word;
                d->code_word = h->code_word;
                d->ie = (float)acc[2 * CM_E];
                d->qe = (float)acc[2 * CM_E + 1];
                d->ip = (float)acc[2 * CM_P];
                d->qp = (float)acc[2 * CM_P + 1];
                d->il = (float)acc[2 * CM_L];
                d->ql = (float)acc[2 * CM_L + 1];
                d->ive = (float)acc[2 * CM_VE];
                d->qve = (float)acc[2 * CM_VE + 1];
                d->ivl = (float)acc[2 * CM_VL];
                d->qvl = (float)acc[2 * CM_VL + 1];
                d->id = (float)acc[2 * CM_D];
                d->qd = (float)acc[2 * CM_D + 1];
            }
            h->seq = (h->seq + 1) & CORR_SEQ_MASK;
            memset(acc, 0, sizeof(acc));
            h->carr_word = next_carr;
            h->code_word = next_code;
        }
    }
    h->carr_phase = ph;
    h->carr_cycles = cyc;
    h->code_phase = cp;
    memcpy(h->acc, acc, sizeof(acc));
    return nd;
}

int corr_model_process(corr_model_t *m, uint64_t t0, const uint8_t *codes, size_t n, corr_dump_t *dumps, int max)
{
    int nd = 0;
    for (int ch = 0; ch < CORR_MAX_CH; ch++) {
        cm_ch_t *h = &m->ch[ch];
        size_t k0 = 0;
        if (h->start_pending) {
            int64_t dt = corr_tsamp_diff(h->start.t_start, t0);
            if (dt >= (int64_t)n) {
                continue;  /* starts in a later block */
            }
            k0 = dt > 0 ? (size_t)dt : 0;
            begin(m, h);
        }
        if (!h->active) {
            continue;
        }
        nd += run(m, h, t0, codes, k0, n, dumps + nd, max - nd, (uint8_t)ch);
    }
    return nd;
}
