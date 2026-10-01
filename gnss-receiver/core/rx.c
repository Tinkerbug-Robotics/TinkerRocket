#include "gnss/rx.h"

#include <math.h>
#include <string.h>

#define TWO_POW_32 4294967296.0
#define CODE_ONE   ((double)((uint64_t)1 << CORR_CODE_FRAC_BITS))
#define CHIP_PER_HZ (1.023e6 / GNSS_FREQ_L1_HZ)

void rx_default_cfg(rx_cfg_t *c, double fs, double if_hz)
{
    memset(c, 0, sizeof(*c));
    c->fs = fs;
    c->if_hz = if_hz;
    c->acq_threshold = 4.5f;  /* 10 ms: real detections at 41 dB-Hz score 7.5-11, noise peaks 3.5 */
    c->acq_ms = 10;
    c->acq_interval_s = 5.0f;
    c->tap_chips = 0.25f;
    c->max_ch = CORR_MAX_CH;
}

void rx_init(rx_t *rx, const rx_cfg_t *cfg)
{
    memset(rx, 0, sizeof(*rx));
    rx->cfg = *cfg;
    rx->if_word = (int32_t)llround(cfg->if_hz / cfg->fs * TWO_POW_32);
    rx->code_word0 = (uint64_t)llround(1.023e6 / cfg->fs * CODE_ONE);
    rx->carr_k = (float)(TWO_POW_32 / cfg->fs);
    rx->code_k = (float)(CODE_ONE / cfg->fs);
    for (int p = 0; p <= GPS_MAX_PRN; p++) {
        rx->prn_ch[p] = -1;
    }
    rx->week = -1;
    pvt_default_opt(&rx->pvt_opt);
}

double rx_time(const rx_t *rx, uint64_t t)
{
    return rx->clk_t + ((double)t - (double)rx->clk_n) / rx->cfg.fs;
}

static void take_nav(rx_t *rx, const lnav_t *l)
{
    if (l->eph.valid) {
        gps_eph_t *e = &rx->eph[l->prn];
        if (!e->valid || e->iode != l->eph.iode || e->toe != l->eph.toe) {
            *e = l->eph;
        }
    }
    if (l->iono.valid) {
        rx->iono = l->iono;
    }
    if (l->week >= 0) {
        rx->week = l->week;
    }
}

static void free_channel(rx_t *rx, int ch)
{
    trk_ch_t *c = &rx->ch[ch];
    if (c->prn >= 1 && c->prn <= GPS_MAX_PRN && rx->prn_ch[c->prn] == ch) {
        rx->prn_ch[c->prn] = -1;
    }
    memset(c, 0, sizeof(*c));
    memset(&rx->nco[ch], 0, sizeof(rx->nco[ch]));
}

int rx_tick(rx_t *rx, uint64_t t_now, const corr_dump_t *d, int nd, corr_cmd_t *cmds, int ncap)
{
    uint8_t touched[CORR_MAX_CH] = {0};
    for (int k = 0; k < nd; k++) {
        int ch = d[k].ch;
        trk_ch_t *c = &rx->ch[ch];
        rx_nco_t *n = &rx->nco[ch];
        if (c->state == TRK_OFF) {
            continue;
        }
        uint64_t t_prev = n->have_dump ? n->last_t : n->t_start;
        float T = (float)((double)(d[k].t_samp - t_prev) / rx->cfg.fs);
        /* Carrier phase relative to the IF, exact: words in force times samples. */
        n->adr_fx += (int64_t)(d[k].t_samp - t_prev) * (int64_t)(d[k].carr_word - rx->if_word);
        n->last_t = d[k].t_samp;
        n->last_code_phase = d[k].code_phase;
        n->last_carr_phase = d[k].carr_phase;
        n->last_carr_cycles = d[k].carr_cycles;
        n->cur_carr = n->sent_carr;
        n->cur_code = n->sent_code;
        n->have_dump = 1;
        if (d[k].seq == 0) {
            continue;  /* the partial first period */
        }
        int bit;
        uint32_t bit_period;
        if (trk_update(c, &d[k], T, &bit, &bit_period)) {
            if (lnav_push(&rx->nav[ch], bit, bit_period)) {
                take_nav(rx, &rx->nav[ch]);
            }
        }
        touched[ch] = 1;
    }

    int nc = 0;
    for (int ch = 0; ch < rx->cfg.max_ch && nc < ncap; ch++) {
        trk_ch_t *c = &rx->ch[ch];
        if (c->prn == 0) {
            continue;
        }
        if (c->state == TRK_OFF) {
            corr_cmd_t *s = &cmds[nc++];
            memset(s, 0, sizeof(*s));
            s->type = CORR_CMD_STOP;
            s->ch = (uint8_t)ch;
            free_channel(rx, ch);
            continue;
        }
        if (!touched[ch]) {
            continue;
        }
        int32_t cw;
        uint64_t kw;
        trk_words(c, &cw, &kw);
        if (cw != rx->nco[ch].sent_carr || kw != rx->nco[ch].sent_code) {
            corr_cmd_t *s = &cmds[nc++];
            memset(s, 0, sizeof(*s));
            s->type = CORR_CMD_NCO;
            s->ch = (uint8_t)ch;
            s->carr_word = cw;
            s->code_word = kw;
            rx->nco[ch].sent_carr = cw;
            rx->nco[ch].sent_code = kw;
            rx->nco[ch].sent_t = t_now;
        }
    }
    return nc;
}

int rx_wants_snapshot(const rx_t *rx, uint64_t t_now, int *ms)
{
    if (t_now < rx->next_acq) {
        return 0;
    }
    int free_ch = 0;
    for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
        free_ch += rx->ch[ch].prn == 0;
    }
    *ms = rx->cfg.acq_ms;
    return free_ch > 0;
}

int rx_acquire(rx_t *rx, uint64_t t_now, uint64_t t0, const float *iq, size_t n, float *work, corr_cmd_t *cmds,
               int ncap)
{
    rx->next_acq = t_now + (uint64_t)((double)rx->cfg.acq_interval_s * rx->cfg.fs);
    acq_cfg_t ac;
    memset(&ac, 0, sizeof(ac));
    ac.fs = rx->cfg.fs;
    ac.if_hz = rx->cfg.if_hz;
    ac.n_fft = 2048;
    ac.n_ms = rx->cfg.acq_ms;
    ac.dop_center = 0.0;
    ac.dop_max = 5000.0;
    acq_t a;
    if (acq_prepare(&a, &ac, iq, n, work) != 0) {
        return 0;
    }
    int nc = 0;
    for (int prn = 1; prn <= GPS_MAX_PRN && nc < ncap; prn++) {
        if (rx->prn_ch[prn] >= 0) {
            continue;
        }
        int ch = -1;
        for (int k = 0; k < rx->cfg.max_ch; k++) {
            if (rx->ch[k].prn == 0) {
                ch = k;
                break;
            }
        }
        if (ch < 0) {
            break;
        }
        acq_result_t r;
        acq_search(&a, prn, &r);
        if (r.metric < rx->cfg.acq_threshold) {
            continue;
        }
        acq_refine(&a, &r);

        trk_ch_t *c = &rx->ch[ch];
        trk_start(c, prn, (float)r.dop_hz, rx->cfg.tap_chips, rx->if_word, rx->code_word0, rx->carr_k, rx->code_k);
        c->acq_metric = r.metric;
        lnav_init(&rx->nav[ch], prn);
        rx->prn_ch[prn] = ch;
        int32_t cw;
        uint64_t kw;
        trk_words(c, &cw, &kw);
        /* Carry the code phase from the snapshot's first sample to the start sample. */
        double rate = 1.023e6 * (1.0 + r.dop_hz / GNSS_FREQ_L1_HZ) / rx->cfg.fs;
        double ph = fmod(r.code_phase + (double)(t_now - t0) * rate, (double)GPS_CA_LEN);
        if (ph < 0.0) {
            ph += GPS_CA_LEN;
        }
        rx_nco_t *nn = &rx->nco[ch];
        memset(nn, 0, sizeof(*nn));
        nn->t_start = t_now;
        nn->sent_carr = cw;
        nn->sent_code = kw;
        nn->sent_t = t_now;
        nn->cur_carr = cw;
        nn->cur_code = kw;

        corr_cmd_t *s = &cmds[nc++];
        memset(s, 0, sizeof(*s));
        s->type = CORR_CMD_START;
        s->ch = (uint8_t)ch;
        s->sig = GNSS_SIG_GPS_L1CA;
        s->prn = (uint8_t)prn;
        s->t_start = t_now;
        s->code_phase = (uint64_t)llround(ph * CODE_ONE);
        s->tap_offset = (uint64_t)llround((double)rx->cfg.tap_chips * CODE_ONE);
        s->carr_word = cw;
        s->code_word = kw;
    }
    return nc;
}

int rx_measure(rx_t *rx, uint64_t t, rx_obs_t *obs, int max, pvt_sol_t *sol)
{
    int no = 0;
    double t_tx[CORR_MAX_CH];
    for (int ch = 0; ch < rx->cfg.max_ch && no < max; ch++) {
        const trk_ch_t *c = &rx->ch[ch];
        const rx_nco_t *n = &rx->nco[ch];
        const lnav_t *l = &rx->nav[ch];
        if (c->state != TRK_LOCKED || !l->synced || !n->have_dump || !rx->eph[c->prn].valid) {
            continue;
        }
        /* The period opened by the last dump is c->period + 1; whole periods since the subframe began. */
        int64_t periods = (int64_t)c->period + 1 - (int64_t)l->sf_period;
        double chips = ((double)n->last_code_phase + (double)(t - n->last_t) * (double)n->cur_code) / CODE_ONE;
        double tt = l->sf_tow + (double)periods * 1e-3 + chips / 1.023e6;
        if (tt >= 604800.0) {
            tt -= 604800.0;
        }
        int64_t adr_fx = n->adr_fx + (int64_t)(t - n->last_t) * (int64_t)(n->cur_carr - rx->if_word);
        rx_obs_t *o = &obs[no];
        o->prn = c->prn;
        o->ch = ch;
        o->t_sv = tt;
        o->dop = (double)(n->cur_carr - rx->if_word) * rx->cfg.fs / TWO_POW_32;
        /* RINEX phase grows with range: the negative of the NCO's accumulated Doppler phase; the
         * Costas half cycle is resolved by the preamble's polarity. */
        o->adr = -(double)adr_fx / TWO_POW_32 + (l->inverted ? 0.5 : 0.0);
        o->cn0 = c->cn0;
        o->lock_s = c->t_state;
        o->half_cycle = 1;
        t_tx[no] = tt;
        no++;
    }
    memset(sol, 0, sizeof(*sol));
    if (no < 4) {
        return no;
    }
    if (!rx->clk_valid) {
        /* First fix: guess the receiver clock from the latest transmit time plus a typical 75 ms. */
        double mx = t_tx[0];
        for (int k = 1; k < no; k++) {
            if (gps_time_diff(t_tx[k], mx) > 0.0) {
                mx = t_tx[k];
            }
        }
        rx->clk_t = fmod(mx + 0.075, 604800.0);
        rx->clk_n = t;
        rx->clk_valid = 1;
    }
    double t_rx = rx_time(rx, t);
    pvt_meas_t m[PVT_MAX_SAT];
    int nm = 0;
    for (int k = 0; k < no && nm < PVT_MAX_SAT; k++) {
        obs[k].pr = gps_time_diff(t_rx, obs[k].t_sv) * GNSS_C;
        m[nm].prn = obs[k].prn;
        m[nm].pr = obs[k].pr;
        m[nm].dop = obs[k].dop;
        m[nm].t_sv = obs[k].t_sv;
        m[nm].cn0 = obs[k].cn0;
        nm++;
    }
    const double *pos0 = rx->sol.valid ? rx->sol.pos : NULL;
    if (pvt_solve(m, nm, rx->eph, &rx->iono, &rx->pvt_opt, pos0, sol) == 0) {
        /* Steer the receiver clock onto GPS time when it is off by more than a microsecond. */
        if (fabs(sol->clk_bias) > 300.0) {
            double dt = sol->clk_bias / GNSS_C;
            rx->clk_t = fmod(rx->clk_t - dt + 604800.0, 604800.0);
            for (int k = 0; k < no; k++) {
                obs[k].pr -= sol->clk_bias;
            }
            sol->clk_bias = 0.0;
        }
        rx->sol = *sol;
    }
    return no;
}
