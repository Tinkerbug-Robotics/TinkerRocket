#include "gnss/rx.h"

#include <math.h>
#include <stdlib.h>
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
    c->hatch_s = 100.0f;
    c->pvt_weights = 1;
    c->adapt_tau_s = 30.0f;
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
    rx->week_ref = LNAV_WEEK_REF;
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

static void predict_at(const rx_t *rx, const pvt_sol_t *s, const gps_eph_t *e, double t_rx, double *t_sv, double *dop,
                       double *el, double los[3]);

static int cmp_double(const void *a, const void *b)
{
    const double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

void rx_set_week_ref(rx_t *rx, int week)
{
    rx->week_ref = week;
}

void rx_set_seed(rx_t *rx, const double pos_ecef[3], double pos_sigma_m, int week, double tow, double tow_sigma_s,
                 uint64_t t)
{
    pvt_sol_t *s = &rx->seed;
    memset(s, 0, sizeof(*s));
    memcpy(s->pos, pos_ecef, sizeof(s->pos));
    ecef_to_geo(s->pos, &s->lat, &s->lon, &s->h);
    rx->seed_pos_sigma = pos_sigma_m;
    rx->seed_tow_sigma = tow_sigma_s;
    rx->seed_valid = 1;
    if (!rx->clk_valid) {
        rx->clk_t = tow;
        rx->clk_n = t;
        rx->clk_valid = 1;
        /* Good to 0.125 ms, prediction errors included, the millisecond rounds safely (4 sigma). */
        rx->time_coarse = tow_sigma_s + pos_sigma_m / GNSS_C > 1.25e-4;
    }
    rx->week_ref = week;
    if (rx->week < 0) {
        rx->week = week;
    }
}

/*
 * GPS channels' whole milliseconds from a prediction (the code phase gives the rest):
 *   - before the first fix, from the seed. Differences against a reference satellite (one already
 *     resolved, else the first) are what is rounded, so the seed clock's error, common to all,
 *     cancels; a channel keeps its millisecond once set. A coarse seed time leaves them all one
 *     whole-millisecond offset out together, which the fixes solve;
 *   - after it, against the fix's own position and time (the clock steering keeps a coarse
 *     offset in the receiver time too, so the rounding returns the same milliseconds).
 * Then the navigation messages vote: where two or more satellites' message times agree on an
 * offset from these milliseconds, and outnumber those that agree with them, the offset is the
 * receiver's, and every channel and the clock move by it (the pseudoranges stay as they are);
 * this settles coarse time. Once the time is settled a satellite whose message time still
 * disagrees has its bit sync wrong (it decodes, shifted), so its bit sync and decoder restart.
 * Lines of sight for the aiding come from the seed until there is a fix.
 */
static void resolve_ms(rx_t *rx, uint64_t t)
{
    const int have_fix = rx->sol.valid;
    if (!rx->clk_valid || (!have_fix && !rx->seed_valid)) {
        return;
    }
    const pvt_sol_t *st = have_fix ? &rx->sol : &rx->seed;
    const double t_rx = rx_time(rx, t) - (have_fix ? rx->sol.clk_bias / GNSS_C : 0.0);
    const double week_ms = 604800000.0;
    int have_ref = 0;
    double ref_d = 0.0;
    int64_t ref_n = 0;
    double dms[CORR_MAX_CH];
    int ok[CORR_MAX_CH];
    for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
        ok[ch] = 0;
        trk_ch_t *c = &rx->ch[ch];
        rx_nco_t *n = &rx->nco[ch];
        if (c->state == TRK_OFF || n->sec_len > 0 || !n->have_dump || sys_of(c->sig) != GNSS_SYS_GPS) {
            continue;
        }
        const gps_eph_t *e = &rx->eph[GNSS_SYS_GPS][c->prn];
        if (!e->valid) {
            continue;
        }
        double t_sv, dop, el;
        predict_at(rx, st, e, t_rx, &t_sv, &dop, &el, have_fix ? NULL : n->los);
        if (!have_fix) {
            n->have_los = 1;
        }
        if (!c->locked_once || c->t_tracked < 1.0f || c->cn0_lin < 1000.0f) {
            continue;  /* the code must sit on its peak: a confirmed PLL, a second, 30 dB-Hz */
        }
        const double chips = ((double)n->last_code_phase + (double)(t - n->last_t) * (double)n->cur_code) / CODE_ONE;
        dms[ch] = (t_sv - chips / 1.023e6) * 1000.0 - (double)c->period;
        ok[ch] = 1;
        if (!have_fix && n->ms_valid && !have_ref) {
            have_ref = 1;
            ref_d = dms[ch];
            ref_n = n->n1;
        }
    }
    for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
        if (!ok[ch]) {
            continue;
        }
        rx_nco_t *n = &rx->nco[ch];
        int64_t n1;
        if (have_fix) {
            n1 = llround(dms[ch]);
        } else {
            if (n->ms_valid) {
                continue;
            }
            if (!have_ref) {
                have_ref = 1;
                ref_d = dms[ch];
                ref_n = llround(dms[ch]);
            }
            double dd = dms[ch] - ref_d;
            if (dd > 0.5 * week_ms) {
                dd -= week_ms;
            } else if (dd < -0.5 * week_ms) {
                dd += week_ms;
            }
            n1 = ref_n + llround(dd);
        }
        if (!n->ms_valid || n->n1 != n1) {
            n->n1 = n1;
            n->t_code = 1e-3;
            n->ms_valid = 1;
            rx->n_ms_fixed++;
        }
    }

    /* The vote: each decoded message's offset from the resolved millisecond. */
    int64_t dn[CORR_MAX_CH];
    int vch[CORR_MAX_CH], nv = 0;
    for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
        const rx_nco_t *n = &rx->nco[ch];
        const lnav_t *l = &rx->nav[ch];
        if (rx->ch[ch].state == TRK_OFF || n->sec_len > 0 || !n->ms_valid || !l->synced) {
            continue;
        }
        const int64_t wk = (int64_t)week_ms;
        int64_t d = (llround(l->sf_tow * 1000.0) + 1 - (int64_t)l->sf_period - n->n1) % wk;
        d = d >= wk / 2 ? d - wk : (d < -wk / 2 ? d + wk : d);
        dn[nv] = d;
        vch[nv++] = ch;
    }
    int64_t mode = 0;
    int n_mode = 0, n_zero = 0;
    for (int i = 0; i < nv; i++) {
        int cnt = 0;
        for (int j = 0; j < nv; j++) {
            cnt += dn[j] == dn[i];
        }
        n_zero += dn[i] == 0;
        if (cnt > n_mode) {
            n_mode = cnt;
            mode = dn[i];
        }
    }
    /* A coarse time takes two agreeing satellites; a settled one, three and twice those
     * against (a seed that claimed better than it was). */
    if (mode != 0 && (rx->time_coarse ? n_mode >= 2 && n_mode > n_zero : n_mode >= 3 && n_mode > 2 * n_zero)) {
        for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
            if (rx->nco[ch].ms_valid && rx->nco[ch].sec_len == 0) {
                rx->nco[ch].n1 += mode;
            }
        }
        rx->clk_t = fmod(rx->clk_t + 1e-3 * (double)mode + 604800.0, 604800.0);
        rx->anchor_ms += mode;
        rx->retimed = 1;  /* (a pilot started on the old time has its code and secondary code out) */
        for (int i = 0; i < nv; i++) {
            dn[i] -= mode;
        }
        rx->time_coarse = 0;
        rx->t_anchor = t;
    } else if (rx->time_coarse && mode == 0 && n_mode >= 2) {
        rx->time_coarse = 0;  /* the seed's millisecond was right */
        rx->t_anchor = t;
    }
    if (rx->time_coarse) {
        return;
    }
    for (int i = 0; i < nv; i++) {
        if (dn[i] == 0) {
            continue;
        }
        const int ch = vch[i];
        trk_ch_t *c = &rx->ch[ch];
        lnav_t *l = &rx->nav[ch];
        const int prn = l->prn, ref = rx->week_ref;
        lnav_init(l, prn);
        l->week_ref = ref;
        c->bit_sync = 0;
        memset(c->hist, 0, sizeof(c->hist));
        c->n_trans = 0;
        rx->n_nav_reset++;
    }
}

void rx_set_accel(rx_t *rx, const double acc_ecef[3], int valid)
{
    rx->acc_valid = valid != 0;
    if (valid) {
        memcpy(rx->acc, acc_ecef, sizeof(rx->acc));
    }
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
        if (rx->acc_valid && n->have_los) {
            /* The acceleration along the line of sight closes the range: +a.u / lambda Hz/s. The
             * commands from this dump land cmd_lead + 1 periods on. */
            const double a_los = rx->acc[0] * n->los[0] + rx->acc[1] * n->los[1] + rx->acc[2] * n->los[2];
            c->ff_rate = (float)(a_los / (GNSS_C / GNSS_FREQ_L1_HZ));
            c->ff_lead = (float)(rx->cfg.cmd_lead + 1) * T;
        } else {
            c->ff_rate = 0.0f;
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

    if (rx->retimed) {
        for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
            if (rx->ch[ch].prn != 0 && rx->nco[ch].sec_len > 0) {
                rx->ch[ch].state = TRK_OFF;
            }
        }
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
    if (rx->retimed) {
        /* Once every pilot is stopped, the holds their failures set on the old time go too. */
        int left = 0;
        for (int ch = 0; ch < rx->cfg.max_ch; ch++) {
            left += rx->ch[ch].prn != 0 && rx->nco[ch].sec_len > 0;
        }
        if (!left) {
            memset(rx->aid_hold, 0, sizeof(rx->aid_hold));
            rx->retimed = 0;
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
        rx->nav[ch].week_ref = rx->week_ref;
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
    resolve_ms(rx, t);
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
            if (!c->locked_once || c->cn0_lin < 1000.0f) {
                /* An aided start counts once its PLL has confirmed the signal, and while the
                 * signal stays above the moments estimate's noise floor (30 dB-Hz). */
                continue;
            }
            /* A pilot started by rx_aid: the period opened by the last dump, c->period + 1,
             * began at code period n1 + c->period of the week. */
            tt = (double)(n->n1 + (int64_t)c->period) * n->t_code + chips / 1.023e6;
        } else {
            /* The period opened by the last dump is c->period + 1. Its start: from the seed's or a
             * fix's millisecond when resolved (resolve_ms keeps the message honest), else from
             * the navigation message, whole periods since its subframe began. */
            int64_t n1;
            if (n->ms_valid) {
                n1 = n->n1;
            } else if (l->synced && !rx->time_coarse) {
                n1 = llround(l->sf_tow * 1000.0) + 1 - (int64_t)l->sf_period;
            } else {
                continue;  /* (a message's time is true; a coarse receiver's is offset) */
            }
            tt = (double)(n1 + (int64_t)c->period) * 1e-3 + chips / 1.023e6;
            tt = fmod(tt, 604800.0);
            if (tt < 0.0) {
                tt += 604800.0;
            }
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
                /* The loop's own rate, plus the IMU's feed-forward when aided (x1 then holds only the
                 * residual). */
                const double rate = (double)c->x1 / TWO_PI + (double)c->ff_rate;
                o->dop = (double)(adr_fx - a0) / TWO_POW_32 / span + rate * 0.5 * span;
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
    if (no >= 3) {
        /* Every satellite's transmit time lies within ~0.1 s of the others' (paths differ by under
         * 30 ms); one far off took its time from a false frame sync (a data word that began like a
         * preamble). Unsync it and leave it out. */
        double d[CORR_MAX_CH], srt[CORR_MAX_CH];
        for (int k = 0; k < no; k++) {
            d[k] = srt[k] = gps_time_diff(t_tx[k], t_tx[0]);
        }
        qsort(srt, (size_t)no, sizeof(double), cmp_double);
        const double med = (no & 1) ? srt[no / 2] : 0.5 * (srt[no / 2 - 1] + srt[no / 2]);
        int w = 0;
        for (int k = 0; k < no; k++) {
            if (fabs(d[k] - med) > 0.1) {
                if (rx->nco[obs[k].ch].sec_len == 0) {
                    rx->nav[obs[k].ch].synced = 0;
                }
                continue;
            }
            obs[w] = obs[k];
            t_tx[w] = t_tx[k];
            w++;
        }
        no = w;
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
    float sig_model[PVT_MAX_SAT];
    int nm = 0;
    const double lambda = GNSS_C / GNSS_FREQ_L1_HZ;
    for (int k = 0; k < no && nm < PVT_MAX_SAT; k++) {
        obs[k].pr_raw = obs[k].pr = gps_time_diff(t_rx, t_tx[k]) * GNSS_C;
        /* Carrier smoothing: the code's noise averaged over hatch_s, the carrier carrying the
         * range change between epochs. Restarted whenever the PLL lets go or the Costas half
         * cycle is resolved afresh. */
        rx_nco_t *n = &rx->nco[obs[k].ch];
        const double adr_m = obs[k].adr * lambda;
        const int inv = n->sec_len == 0 && rx->nav[obs[k].ch].inverted;
        if (rx->cfg.hatch_s > 0.0f && obs[k].lock_s > 0.0f && n->hatch_n > 0 && inv == n->hatch_inv) {
            double dt = (double)(t - n->hatch_t) / rx->cfg.fs;
            double mx = dt > 0.0 ? (double)rx->cfg.hatch_s / dt : 1.0;
            double w = (double)(n->hatch_n + 1) < mx ? (double)(n->hatch_n + 1) : mx;
            n->hatch_pr = obs[k].pr / w + (1.0 - 1.0 / w) * (n->hatch_pr + (adr_m - n->hatch_adr));
            n->hatch_n++;
        } else {
            n->hatch_pr = obs[k].pr;
            n->hatch_n = obs[k].lock_s > 0.0f ? 1 : 0;
            n->hatch_t0 = t;
        }
        n->hatch_adr = adr_m;
        n->hatch_t = t;
        n->hatch_inv = inv;
        float smooth_s = 0.0f;
        if (rx->cfg.hatch_s > 0.0f && n->hatch_n > 0) {
            obs[k].pr = n->hatch_pr;
            smooth_s = (float)((double)(t - n->hatch_t0) / rx->cfg.fs);
        }
        m[nm].sys = obs[k].sys;
        m[nm].prn = obs[k].prn;
        m[nm].pr = obs[k].pr;
        m[nm].dop = obs[k].dop;
        m[nm].t_sv = obs[k].t_sv;
        m[nm].cn0 = obs[k].cn0;
        /* Weights for the fix: the code's noise at this C/N0, as far as smoothing has brought it
         * down; the Doppler's, from the loop it came through (a pilot's capped as trk caps it). */
        const trk_ch_t *c = &rx->ch[obs[k].ch];
        const int locked = obs[k].lock_s > 0.0f;
        float bw = locked ? rx->prof.locked.pll_bw : rx->prof.pullin.pll_bw;
        if (c->pilot) {
            const float cap = 0.05f / (c->sig == GNSS_SIG_GAL_E1C ? 0.004f : 0.010f);
            bw = bw < cap ? bw : cap;
        }
        float sg = (float)pvt_sigma_pr(obs[k].cn0, smooth_s);
        sig_model[nm] = sg;
        if (rx->cfg.adapt_tau_s > 0.0f && n->res_var > 1.0f) {
            /* A satellite whose residuals run larger than its model (a code that wanders with the
             * sample phase, say) counts for less, rather than being left out one epoch in three. */
            sg *= n->res_var < 25.0f ? sqrtf(n->res_var) : 5.0f;
        }
        m[nm].sigma = rx->cfg.pvt_weights ? sg : 1.0f;
        m[nm].sigma_dop = rx->cfg.pvt_weights ? (float)pvt_sigma_dop(obs[k].cn0, locked, bw) : 1.0f;
        nm++;
    }
    const double *pos0 = rx->sol.valid ? rx->sol.pos : (rx->seed_valid ? rx->seed.pos : NULL);
    pvt_opt_t opt = rx->pvt_opt;
    opt.coarse_time = rx->time_coarse;
    const int fix = pvt_solve(m, nm, rx->eph, &rx->iono, &opt, pos0, sol);
    if (rx->cfg.adapt_tau_s > 0.0f && rx->sol.valid && fix != -1) {
        /* Learn each satellite's residual spread against its model sigma. One left out by the
         * test counts with the residual it was left out for. */
        for (int k = 0; k < nm; k++) {
            if (!sol->used[k] && !(sol->excluded[k] & 1)) {
                continue;
            }
            rx_nco_t *n = &rx->nco[obs[k].ch];
            const double dt = (double)(t - n->res_t) / rx->cfg.fs, tau = (double)rx->cfg.adapt_tau_s;
            const float a = n->res_var > 0.0f && dt > 0.0 && dt < tau ? (float)(dt / tau) : 1.0f;
            const float z = (float)(sol->resid[k] / (double)sig_model[k]);
            const float z2 = z * z < 100.0f ? z * z : 100.0f;
            n->res_var = n->res_var > 0.0f ? n->res_var + a * (z2 - n->res_var) : (z2 > 1.0f ? z2 : 1.0f);
            n->res_t = t;
        }
    }
    if (fix == 0) {
        /* Steer the receiver clock onto GPS time when it is off by more than a microsecond. */
        if (fabs(sol->clk_bias) > 300.0) {
            double dt = sol->clk_bias / GNSS_C;
            rx->clk_t = fmod(rx->clk_t - dt + 604800.0, 604800.0);
            for (int k = 0; k < no; k++) {
                obs[k].pr -= sol->clk_bias;
                obs[k].pr_raw -= sol->clk_bias;
                rx->nco[obs[k].ch].hatch_pr -= sol->clk_bias;  /* the smoothing follows the step */
            }
            sol->clk_bias = 0.0;
        }
        rx->sol = *sol;
        for (int k = 0; k < no; k++) {
            rx_nco_t *n = &rx->nco[obs[k].ch];
            const gps_eph_t *e = &rx->eph[obs[k].sys][obs[k].prn];
            if (sol->used[k]) {
                memcpy(n->los, sol->los[k], sizeof(n->los));
                n->have_los = 1;
            } else if (e->valid) {
                /* Left out of the fix (unhealthy, under the mask, rejected), a satellite is still
                 * tracked, and its line of sight from the fix still carries the aiding. */
                double p[3], clk;
                gps_sat_pos(e, obs[k].t_sv, p, NULL, &clk);
                const double d[3] = {p[0] - sol->pos[0], p[1] - sol->pos[1], p[2] - sol->pos[2]};
                const double r = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                for (int j = 0; j < 3; j++) {
                    n->los[j] = d[j] / r;
                }
                n->have_los = 1;
            }
        }
    }
    return no;
}

/* What the fix predicts for satellite e at GPS time t_rx (s of week, true): the transmit time on
 * the satellite's clock (its system's time), the Doppler (Hz) and the elevation (rad). */
static void predict_at(const rx_t *rx, const pvt_sol_t *s, const gps_eph_t *e, double t_rx, double *t_sv, double *dop,
                       double *el, double los[3])
{
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
    if (los) {
        memcpy(los, u, sizeof(u));
    }
}

static void predict(const rx_t *rx, const gps_eph_t *e, double t_rx, double *t_sv, double *dop, double *el)
{
    predict_at(rx, &rx->sol, e, t_rx, t_sv, dop, el, NULL);
}

int rx_aid(rx_t *rx, uint64_t t_now, corr_cmd_t *cmds, int ncap)
{
    if (!rx->sol.valid || !rx->clk_valid || rx->time_coarse || t_now < rx->next_aid) {
        return 0;  /* (a coarse time puts a pilot's code whole milliseconds out) */
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
