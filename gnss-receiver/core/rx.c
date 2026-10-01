#include "gnss/rx.h"

#include <math.h>
#include <string.h>

#define TWO_POW_32 4294967296.0
#define CODE_ONE   ((double)((uint64_t)1 << CORR_CODE_FRAC_BITS))
#define CHIP_PER_HZ (1.023e6 / GNSS_FREQ_L1_HZ)
#define TWO_PI     6.283185307179586476925286766559

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
    c->cmd_lead = 2;
    c->quiet = trk_profile_quiet;
    c->boost = trk_profile_boost;
}

void rx_init(rx_t *rx, const rx_cfg_t *cfg)
{
    memset(rx, 0, sizeof(*rx));
    rx->cfg = *cfg;
    rx->if_word = (int32_t)llround(cfg->if_hz / cfg->fs * TWO_POW_32);
    rx->code_word0 = (uint64_t)llround(1.023e6 / cfg->fs * CODE_ONE);
    rx->carr_k = (float)(TWO_POW_32 / cfg->fs);
    rx->code_k = (float)(CODE_ONE / cfg->fs);
    for (int sys = 0; sys < GNSS_SYS_COUNT; sys++) {
        for (int p = 0; p <= GNSS_MAX_PRN; p++) {
            rx->sat_ch[sys][p] = -1;
        }
    }
    rx->week = -1;
    rx->prof = cfg->quiet;
    pvt_default_opt(&rx->pvt_opt);
}

double rx_time(const rx_t *rx, uint64_t t)
{
    return rx->clk_t + ((double)t - (double)rx->clk_n) / rx->cfg.fs;
}

static void take_nav(rx_t *rx, const lnav_t *l)
{
    if (l->eph.valid) {
        gps_eph_t *e = &rx->eph[GNSS_SYS_GPS][l->prn];
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

static int sys_of(int sig)
{
    return sig == GNSS_SIG_GAL_E1C ? GNSS_SYS_GAL : (sig == GNSS_SIG_BDS_B1CP ? GNSS_SYS_BDS : GNSS_SYS_GPS);
}

static void free_channel(rx_t *rx, int ch)
{
    trk_ch_t *c = &rx->ch[ch];
    const int sys = sys_of(c->sig);
    if (c->prn >= 1 && c->prn <= GNSS_MAX_PRN && rx->sat_ch[sys][c->prn] == ch) {
        rx->sat_ch[sys][c->prn] = -1;
    }
    memset(c, 0, sizeof(*c));
    memset(&rx->nco[ch], 0, sizeof(rx->nco[ch]));
}

void rx_set_boost(rx_t *rx, int on)
{
    rx->boost = on != 0;
}

int rx_tick(rx_t *rx, uint64_t t_now, const corr_dump_t *d, int nd, corr_cmd_t *cmds, int ncap)
{
    trk_profile_step(&rx->prof, rx->boost ? &rx->cfg.boost : &rx->cfg.quiet,
                     (float)((double)(t_now - rx->t_tick) / rx->cfg.fs));
    rx->t_tick = t_now;
    const trk_profile_t *prof = &rx->prof;
    uint8_t touched[CORR_MAX_CH] = {0};
    for (int k = 0; k < nd; k++) {
        int ch = d[k].ch;
        trk_ch_t *c = &rx->ch[ch];
        rx_nco_t *n = &rx->nco[ch];
        if (c->state == TRK_OFF) {
            continue;
        }
        /* Extend the correlator's wrapping counters: the sample to 64 bits (a dump is never
         * newer than the tick), the period to 32. Everything below works in the extended counts. */
        corr_dump_t x = d[k];
        x.t_samp = t_now - ((t_now - d[k].t_samp) & CORR_TSAMP_MASK);
        x.seq = n->have_dump ? n->period + (uint32_t)corr_seq_diff(d[k].seq, n->period) : d[k].seq;
        n->period = x.seq;
        uint64_t t_prev = n->have_dump ? n->last_t : n->t_start;
        float T = (float)((double)(x.t_samp - t_prev) / rx->cfg.fs);
        /* Carrier phase relative to the IF, exact: words in force times samples. */
        n->adr_fx += (int64_t)(x.t_samp - t_prev) * (int64_t)(x.carr_word - rx->if_word);
        n->hist_adr[n->hist_head] = n->adr_fx;
        n->hist_t[n->hist_head] = x.t_samp;
        n->hist_head = (n->hist_head + 1) % RX_DOP_DUMPS;
        n->hist_n += n->hist_n < RX_DOP_DUMPS;
        n->last_t = x.t_samp;
        n->last_code_phase = x.code_phase;
        n->last_carr_phase = x.carr_phase;
        n->last_carr_cycles = x.carr_cycles;
        /* Words for the period this epoch opens: the dump's own, unless a command's tag has come. */
        n->cur_carr = x.carr_word;
        n->cur_code = x.code_word;
        while (n->npend > 0 && n->pend_seq[0] <= x.seq) {
            n->cur_carr = n->pend_carr[0];
            n->cur_code = n->pend_code[0];
            for (int j = 1; j < n->npend; j++) {
                n->pend_seq[j - 1] = n->pend_seq[j];
                n->pend_carr[j - 1] = n->pend_carr[j];
                n->pend_code[j - 1] = n->pend_code[j];
            }
            n->npend--;
        }
        n->have_dump = 1;
        if (x.seq == 0) {
            continue;  /* the partial first period */
        }
        if (n->sec_len > 0 && !c->locked_once && c->t_tracked > 5.0f) {
            c->state = TRK_OFF;  /* an aided start that never locked: no signal where predicted */
            continue;
        }
        if (n->sec_len > 0) {
            /* A pilot: period k (k >= 1) carries secondary chip (n1 + k - 1) mod its length. */
            if (n->sec[(uint64_t)(n->n1 + (int64_t)x.seq - 1) % n->sec_len]) {
                x.ie = -x.ie, x.qe = -x.qe, x.ip = -x.ip, x.qp = -x.qp, x.il = -x.il, x.ql = -x.ql;
                x.ive = -x.ive, x.qve = -x.qve, x.ivl = -x.ivl, x.qvl = -x.qvl;
            }
        }
        int bit;
        uint32_t bit_period;
        if (trk_update(c, prof, &x, T, &bit, &bit_period)) {
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
            if (rx->nco[ch].sec_len > 0) {
                /* An aided start that found no signal (or lost it): leave it 30 s before the next. */
                rx->aid_hold[sys_of(c->sig)][c->prn] = t_now + (uint64_t)llround(30.0 * rx->cfg.fs);
            }
            free_channel(rx, ch);
            continue;
        }
        if (!touched[ch]) {
            continue;
        }
        int32_t cw;
        uint64_t kw;
        trk_words(c, &cw, &kw);
        rx_nco_t *n = &rx->nco[ch];
        if (cw != n->sent_carr || kw != n->sent_code) {
            uint32_t tag = c->period + rx->cfg.cmd_lead;
            corr_cmd_t *s = &cmds[nc++];
            memset(s, 0, sizeof(*s));
            s->type = CORR_CMD_NCO;
            s->ch = (uint8_t)ch;
            s->carr_word = cw;
            s->code_word = kw;
            s->apply_seq = tag & CORR_SEQ_MASK;
            n->sent_carr = cw;
            n->sent_code = kw;
            /* Same tag replaces (as the correlator does); else append, oldest dropped if full. */
            if (n->npend > 0 && n->pend_seq[n->npend - 1] == tag) {
                n->npend--;
            } else if (n->npend == 4) {
                for (int j = 1; j < 4; j++) {
                    n->pend_seq[j - 1] = n->pend_seq[j];
                    n->pend_carr[j - 1] = n->pend_carr[j];
                    n->pend_code[j - 1] = n->pend_code[j];
                }
                n->npend--;
            }
            n->pend_seq[n->npend] = tag;
            n->pend_carr[n->npend] = cw;
            n->pend_code[n->npend] = kw;
            n->npend++;
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
        if (rx->sat_ch[GNSS_SYS_GPS][prn] >= 0) {
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
        rx->sat_ch[GNSS_SYS_GPS][prn] = ch;
        int32_t cw;
        uint64_t kw;
        trk_words(c, &cw, &kw);
        /* Start a whole tick ahead, so the command arrives in time whatever the P4's latency, and
         * carry the code phase from the snapshot's first sample to that start sample. */
        uint64_t t_start = t_now + (uint64_t)llround(rx->cfg.fs * 1e-3);
        double rate = 1.023e6 * (1.0 + r.dop_hz / GNSS_FREQ_L1_HZ) / rx->cfg.fs;
        double ph = fmod(r.code_phase + (double)(t_start - t0) * rate, (double)GPS_CA_LEN);
        if (ph < 0.0) {
            ph += GPS_CA_LEN;
        }
        rx_nco_t *nn = &rx->nco[ch];
        memset(nn, 0, sizeof(*nn));
        nn->t_start = t_start;
        nn->sent_carr = cw;
        nn->sent_code = kw;
        nn->cur_carr = cw;
        nn->cur_code = kw;

        corr_cmd_t *s = &cmds[nc++];
        memset(s, 0, sizeof(*s));
        s->type = CORR_CMD_START;
        s->ch = (uint8_t)ch;
        s->sig = GNSS_SIG_GPS_L1CA;
        s->prn = (uint8_t)prn;
        s->t_start = t_start & CORR_TSAMP_MASK;
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
    double t_tx[CORR_MAX_CH];  /* in GPS time */
    for (int ch = 0; ch < rx->cfg.max_ch && no < max; ch++) {
        const trk_ch_t *c = &rx->ch[ch];
        const rx_nco_t *n = &rx->nco[ch];
        const lnav_t *l = &rx->nav[ch];
        const int sys = sys_of(c->sig);
        /* A channel the PLL has let go of still measures: its code and frequency hold under the
         * FLL. Only its carrier phase is void, and lock_s = 0 says so. */
        if (c->state == TRK_OFF || !n->have_dump || !rx->eph[sys][c->prn].valid) {
            continue;
        }
        double chips = ((double)n->last_code_phase + (double)(t - n->last_t) * (double)n->cur_code) / CODE_ONE;
        double tt;
        if (n->sec_len > 0) {
            if (!c->locked_once) {
                continue;  /* an aided start counts only once its PLL has confirmed the signal */
            }
            /* A pilot started by rx_aid: the period opened by the last dump, c->period + 1,
             * began at code period n1 + c->period of the week. */
            tt = (double)(n->n1 + (int64_t)c->period) * n->t_code + chips / 1.023e6;
        } else {
            if (!l->synced) {
                continue;
            }
            /* The period opened by the last dump is c->period + 1; whole periods since the subframe began. */
            int64_t periods = (int64_t)c->period + 1 - (int64_t)l->sf_period;
            tt = l->sf_tow + (double)periods * 1e-3 + chips / 1.023e6;
        }
        if (tt >= 604800.0) {
            tt -= 604800.0;
        }
        int64_t adr_fx = n->adr_fx + (int64_t)(t - n->last_t) * (int64_t)(n->cur_carr - rx->if_word);
        rx_obs_t *o = &obs[no];
        o->sys = sys;
        o->sig = c->sig;
        o->prn = c->prn;
        o->ch = ch;
        o->t_sv = tt;
        o->dop = (double)(n->cur_carr - rx->if_word) * rx->cfg.fs / TWO_POW_32;
        if (n->hist_n == RX_DOP_DUMPS) {
            /* The oldest epoch in the history is where the ring's head points. */
            const int64_t a0 = n->hist_adr[n->hist_head];
            const uint64_t t0 = n->hist_t[n->hist_head];
            const double span = (double)(t - t0) / rx->cfg.fs;
            if (span > 0.0) {
                o->dop = (double)(adr_fx - a0) / TWO_POW_32 / span + (double)c->x1 / TWO_PI * 0.5 * span;
            }
        }
        /* RINEX phase grows with range: the negative of the NCO's accumulated Doppler phase; the
         * Costas half cycle is resolved by the preamble's polarity (a pilot has none). */
        o->adr = -(double)adr_fx / TWO_POW_32 + (n->sec_len == 0 && l->inverted ? 0.5 : 0.0);
        o->cn0 = c->cn0;
        o->lock_s = c->state == TRK_LOCKED ? c->t_state : 0.0f;
        o->half_cycle = 1;
        /* BDT runs 14 s behind GPST; GST's seconds of week track GPST's. */
        t_tx[no] = sys == GNSS_SYS_BDS ? tt - BDT_MINUS_GPST : tt;
        no++;
    }
    memset(sol, 0, sizeof(*sol));
    if (no < 4) {
        return no;
    }
    if (!rx->clk_valid) {
        /* First fix: guess the receiver clock from the latest transmit time plus a typical 75 ms. */
        double mx = t_tx[0];  /* GPS time */
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
        obs[k].pr = gps_time_diff(t_rx, t_tx[k]) * GNSS_C;
        m[nm].sys = obs[k].sys;
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

/* What the fix predicts for satellite e at GPS time t_rx (s of week, true): the transmit time on
 * the satellite's clock (its system's time), the Doppler (Hz) and the elevation (rad). */
static void predict(const rx_t *rx, const gps_eph_t *e, double t_rx, double *t_sv, double *dop, double *el)
{
    const pvt_sol_t *s = &rx->sol;
    const double toff = e->sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0;
    double tau = 0.075, pos[3], vel[3], clk = 0.0, c1, p1[3], t_sys = t_rx;
    double az = 0.0, d[3] = {0.0, 0.0, 0.0}, rho = 1.0;
    *el = 0.0;
    for (int it = 0; it < 4; it++) {
        t_sys = t_rx - tau + toff;
        gps_sat_pos(e, t_sys, pos, vel, &clk);
        double a = GPS_OMEGA_E * tau, ca = cos(a), sa = sin(a);
        double sp[3] = {ca * pos[0] + sa * pos[1], -sa * pos[0] + ca * pos[1], pos[2]};
        for (int k = 0; k < 3; k++) {
            d[k] = sp[k] - s->pos[k];
        }
        rho = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        double up[3] = {cos(s->lat) * cos(s->lon), cos(s->lat) * sin(s->lon), sin(s->lat)};
        double east[3] = {-sin(s->lon), cos(s->lon), 0.0};
        double north[3] = {-sin(s->lat) * cos(s->lon), -sin(s->lat) * sin(s->lon), cos(s->lat)};
        double de = (d[0] * east[0] + d[1] * east[1]) / rho;
        double dn = (d[0] * north[0] + d[1] * north[1] + d[2] * north[2]) / rho;
        double du = (d[0] * up[0] + d[1] * up[1] + d[2] * up[2]) / rho;
        *el = asin(du);
        az = atan2(de, dn);
        double delay = 0.0;
        if (rx->iono.valid && rx->pvt_opt.use_iono) {
            delay += iono_klobuchar(&rx->iono, s->lat, s->lon, az, *el, t_rx);
        }
        if (rx->pvt_opt.use_tropo && *el > 0.0) {
            delay += tropo_saastamoinen(s->lat, s->h, *el);
        }
        tau = (rho + delay) / GNSS_C;
    }
    gps_sat_pos(e, t_sys + 1.0, p1, NULL, &c1);
    double u[3] = {d[0] / rho, d[1] / rho, d[2] / rho};
    double rr = (vel[0] - s->vel[0]) * u[0] + (vel[1] - s->vel[1]) * u[1] + (vel[2] - s->vel[2]) * u[2];
    *dop = -(rr + s->clk_drift - GNSS_C * (c1 - clk)) / (GNSS_C / GNSS_FREQ_L1_HZ);
    *t_sv = t_sys + clk;
    if (*t_sv < 0.0) {
        *t_sv += 604800.0;
    }
}

int rx_aid(rx_t *rx, uint64_t t_now, corr_cmd_t *cmds, int ncap)
{
    if (!rx->sol.valid || !rx->clk_valid || t_now < rx->next_aid) {
        return 0;
    }
    rx->next_aid = t_now + (uint64_t)llround(rx->cfg.fs);
    const uint64_t t_start = t_now + (uint64_t)llround(rx->cfg.fs * 1e-3);
    const double t_rx = rx_time(rx, t_start) - rx->sol.clk_bias / GNSS_C;
    const double tap = 0.1, tap2 = 0.5;  /* BOC(1,1): +-0.1 chip on the steep peak, +-0.5 on the side peaks */
    int nc = 0;
    for (int sys = GNSS_SYS_GAL; sys <= GNSS_SYS_BDS && nc < ncap; sys++) {
        for (int prn = 1; prn <= GNSS_MAX_PRN && nc < ncap; prn++) {
            const gps_eph_t *e = &rx->eph[sys][prn];
            if (!e->valid || e->health != 0 || rx->sat_ch[sys][prn] >= 0 || t_now < rx->aid_hold[sys][prn]) {
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
                return nc;
            }
            double t_sv, dop, el;
            predict(rx, e, t_rx, &t_sv, &dop, &el);
            if (el < 10.0 * 3.14159265358979 / 180.0) {
                continue;
            }
            const int sig = sys == GNSS_SYS_GAL ? GNSS_SIG_GAL_E1C : GNSS_SIG_BDS_B1CP;
            const int len = sys == GNSS_SYS_GAL ? GAL_E1_LEN : BDS_B1C_LEN;
            const double t_code = (double)len / 1.023e6;
            const double periods = floor(t_sv / t_code);
            const double ph = (t_sv - periods * t_code) / t_code * (double)len;  /* chips at t_start */

            trk_ch_t *c = &rx->ch[ch];
            trk_start(c, prn, (float)dop, (float)tap, rx->if_word, rx->code_word0, rx->carr_k, rx->code_k);
            trk_set_signal(c, sig);
            memset(&rx->nav[ch], 0, sizeof(rx->nav[ch]));
            rx->sat_ch[sys][prn] = ch;
            int32_t cw;
            uint64_t kw;
            trk_words(c, &cw, &kw);
            rx_nco_t *nn = &rx->nco[ch];
            memset(nn, 0, sizeof(*nn));
            nn->t_start = t_start;
            nn->sent_carr = cw;
            nn->sent_code = kw;
            nn->cur_carr = cw;
            nn->cur_code = kw;
            nn->t_code = t_code;
            nn->n1 = (int64_t)periods + 1;
            if (sys == GNSS_SYS_GAL) {
                gal_e1c_secondary(nn->sec);
                nn->sec_len = GAL_E1C_SEC_LEN;
            } else {
                bds_b1c_secondary(prn, nn->sec);
                nn->sec_len = BDS_B1C_SEC_LEN;
            }

            corr_cmd_t *s = &cmds[nc++];
            memset(s, 0, sizeof(*s));
            s->type = CORR_CMD_START;
            s->ch = (uint8_t)ch;
            s->sig = (uint8_t)sig;
            s->prn = (uint8_t)prn;
            s->t_start = t_start & CORR_TSAMP_MASK;
            s->code_phase = (uint64_t)llround(ph * CODE_ONE);
            s->tap_offset = (uint64_t)llround(tap * CODE_ONE);
            s->tap_offset2 = (uint64_t)llround(tap2 * CODE_ONE);
            s->carr_word = cw;
            s->code_word = kw;
        }
    }
    return nc;
}
