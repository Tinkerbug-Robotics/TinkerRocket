#include "corr_float.h"

#include "corr_model.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.283185307179586476925286766559

void corr_float_init(corr_float_t *c, double fs)
{
    memset(c, 0, sizeof(*c));
    c->fs = fs;
    /* Entry j is the phase at the centre of its bin, so the table has no half-bin bias. */
    for (int j = 0; j < (1 << CF_LUT_BITS); j++) {
        double ph = TWO_PI * ((double)j + 0.5) / (double)(1 << CF_LUT_BITS);
        c->cos_lut[j] = (float)cos(ph);
        c->sin_lut[j] = (float)sin(ph);
    }
}

int corr_float_command(corr_float_t *c, const corr_cmd_t *cmd)
{
    if (cmd->ch >= CORR_MAX_CH) {
        return -1;
    }
    cf_ch_t *h = &c->ch[cmd->ch];
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

/* The channel's codes as +-1, the same chips the bit-exact model uses. */
static int load_codes(cf_ch_t *h, int sig, int prn)
{
    static uint8_t chips[CORR_MAX_CODE_LEN], dchips[CORR_MAX_CODE_LEN];
    int len;
    h->has_data = 0;
    h->boc = 0;
    if (sig == GNSS_SIG_GPS_L1CA) {
        gps_ca_code(prn, chips);
        len = GPS_CA_LEN;
    } else if (sig == GNSS_SIG_GAL_E1C) {
        gal_e1_code(prn, GNSS_SIG_GAL_E1C, chips);
        gal_e1_code(prn, GNSS_SIG_GAL_E1B, dchips);
        h->has_data = h->boc = 1;
        len = GAL_E1_LEN;
    } else {
        bds_b1c_code(prn, GNSS_SIG_BDS_B1CP, chips);
        bds_b1c_code(prn, GNSS_SIG_BDS_B1CD, dchips);
        h->has_data = h->boc = 1;
        len = BDS_B1C_LEN;
    }
    for (int k = 0; k < len; k++) {
        h->code[k] = chips[k] ? -1.0f : 1.0f;
        h->dcode[k] = h->has_data && dchips[k] ? -1.0f : 1.0f;
    }
    return len;
}

static void begin(corr_float_t *c, cf_ch_t *h)
{
    (void)c;
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

/* The replica at code phase x: +-1, negated in the second half of each chip for BOC(1,1). */
static inline float rep_at(const float *code, uint64_t x, int boc)
{
    float v = code[x >> CORR_CODE_FRAC_BITS];
    return (boc && ((x >> (CORR_CODE_FRAC_BITS - 1)) & 1u)) ? -v : v;
}

/* Runs one channel over samples [k0, n) of the block; returns dumps written. */
static int run(corr_float_t *c, cf_ch_t *h, uint64_t t0, const float *iq, size_t k0, size_t n, corr_dump_t *dumps,
               int max, uint8_t ch)
{
    const int shift = 32 - CF_LUT_BITS;
    const float *code = h->code, *dcode = h->dcode;
    const uint64_t mod = h->code_mod, tap = h->tap, tap2 = h->tap2;
    const int boc = h->boc, has_data = h->has_data;
    uint32_t ph = h->carr_phase, cyc = h->carr_cycles;
    uint64_t cp = h->code_phase;
    float a[2 * CF_NTAPS];
    memcpy(a, h->acc, sizeof(a));
    int nd = 0;
    for (size_t k = k0; k < n; k++) {
        uint32_t j = ph >> shift;
        float co = c->cos_lut[j], si = c->sin_lut[j];
        float i = iq[2 * k], q = iq[2 * k + 1];
        /* (i + jq) * exp(-j*phase) */
        float xr = i * co + q * si;
        float xi = q * co - i * si;
        uint64_t xe = cp + tap, xve = cp + tap2;
        if (xe >= mod) {
            xe -= mod;
        }
        if (xve >= mod) {
            xve -= mod;
        }
        uint64_t xl = (cp >= tap) ? cp - tap : cp + mod - tap;
        uint64_t xvl = (cp >= tap2) ? cp - tap2 : cp + mod - tap2;
        const float r[CF_NTAPS] = {rep_at(code, xe, boc), rep_at(code, cp, boc), rep_at(code, xl, boc),
                                   rep_at(code, xve, boc), rep_at(code, xvl, boc),
                                   has_data ? rep_at(dcode, cp, boc) : 0.0f};
        for (int t = 0; t < CF_NTAPS; t++) {
            a[2 * t] += xr * r[t];
            a[2 * t + 1] += xi * r[t];
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
            /* Epoch: the next sample opens a new code period. */
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
                d->ie = a[2 * CF_E];
                d->qe = a[2 * CF_E + 1];
                d->ip = a[2 * CF_P];
                d->qp = a[2 * CF_P + 1];
                d->il = a[2 * CF_L];
                d->ql = a[2 * CF_L + 1];
                d->ive = a[2 * CF_VE];
                d->qve = a[2 * CF_VE + 1];
                d->ivl = a[2 * CF_VL];
                d->qvl = a[2 * CF_VL + 1];
                d->id = a[2 * CF_D];
                d->qd = a[2 * CF_D + 1];
            }
            h->seq = (h->seq + 1) & CORR_SEQ_MASK;
            memset(a, 0, sizeof(a));
            h->carr_word = next_carr;
            h->code_word = next_code;
        }
    }
    h->carr_phase = ph;
    h->carr_cycles = cyc;
    h->code_phase = cp;
    memcpy(h->acc, a, sizeof(a));
    return nd;
}

int corr_float_process(corr_float_t *c, uint64_t t0, const float *iq, size_t n, corr_dump_t *dumps, int max)
{
    int nd = 0;
    for (int ch = 0; ch < CORR_MAX_CH; ch++) {
        cf_ch_t *h = &c->ch[ch];
        size_t k0 = 0;
        if (h->start_pending) {
            int64_t dt = corr_tsamp_diff(h->start.t_start, t0);
            if (dt >= (int64_t)n) {
                continue;  /* starts in a later block */
            }
            k0 = dt > 0 ? (size_t)dt : 0;
            begin(c, h);
        }
        if (!h->active) {
            continue;
        }
        nd += run(c, h, t0, iq, k0, n, dumps + nd, max - nd, (uint8_t)ch);
    }
    return nd;
}
