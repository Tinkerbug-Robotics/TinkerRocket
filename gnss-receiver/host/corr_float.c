#include "corr_float.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.283185307179586476925286766559

void corr_float_init(corr_float_t *c, double fs)
{
    memset(c, 0, sizeof(*c));
    c->fs = fs;
    uint8_t chips[GPS_CA_LEN];
    for (int prn = 1; prn <= GPS_MAX_PRN; prn++) {
        gps_ca_code(prn, chips);
        for (int k = 0; k < GPS_CA_LEN; k++) {
            c->ca[prn - 1][k] = chips[k] ? -1.0f : 1.0f;
        }
    }
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
        if (cmd->sig != GNSS_SIG_GPS_L1CA || cmd->prn < 1 || cmd->prn > GPS_MAX_PRN) {
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

static void begin(corr_float_t *c, cf_ch_t *h)
{
    const corr_cmd_t *s = &h->start;
    h->code = c->ca[s->prn - 1];
    h->code_mod = (uint64_t)GPS_CA_LEN << CORR_CODE_FRAC_BITS;
    h->code_phase = s->code_phase % h->code_mod;
    h->code_word = s->code_word;
    h->tap = s->tap_offset;
    h->carr_word = s->carr_word;
    h->carr_phase = 0;
    h->carr_cycles = 0;
    cmdq_clear(&h->q);
    memset(h->acc, 0, sizeof(h->acc));
    h->seq = 0;
    h->active = 1;
    h->start_pending = 0;
}

/* Runs one channel over samples [k0, n) of the block; returns dumps written. */
static int run(corr_float_t *c, cf_ch_t *h, uint64_t t0, const float *iq, size_t k0, size_t n, corr_dump_t *dumps,
               int max, uint8_t ch)
{
    const int shift = 32 - CF_LUT_BITS;
    const float *code = h->code;
    const uint64_t mod = h->code_mod, tap = h->tap;
    uint32_t ph = h->carr_phase, cyc = h->carr_cycles;
    uint64_t cp = h->code_phase;
    float ie = h->acc[0], qe = h->acc[1], ip = h->acc[2], qp = h->acc[3], il = h->acc[4], ql = h->acc[5];
    int nd = 0;
    for (size_t k = k0; k < n; k++) {
        uint32_t j = ph >> shift;
        float co = c->cos_lut[j], si = c->sin_lut[j];
        float i = iq[2 * k], q = iq[2 * k + 1];
        /* (i + jq) * exp(-j*phase) */
        float xr = i * co + q * si;
        float xi = q * co - i * si;
        uint64_t ce = cp + tap;
        if (ce >= mod) {
            ce -= mod;
        }
        uint64_t cl = (cp >= tap) ? cp - tap : cp + mod - tap;
        float e = code[ce >> CORR_CODE_FRAC_BITS];
        float p = code[cp >> CORR_CODE_FRAC_BITS];
        float l = code[cl >> CORR_CODE_FRAC_BITS];
        ie += xr * e;
        qe += xi * e;
        ip += xr * p;
        qp += xi * p;
        il += xr * l;
        ql += xi * l;

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
                d->ie = ie;
                d->qe = qe;
                d->ip = ip;
                d->qp = qp;
                d->il = il;
                d->ql = ql;
            }
            h->seq = (h->seq + 1) & CORR_SEQ_MASK;
            ie = qe = ip = qp = il = ql = 0.0f;
            h->carr_word = next_carr;
            h->code_word = next_code;
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
