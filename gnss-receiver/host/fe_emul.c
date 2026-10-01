#include "fe_emul.h"

#include "fe_format.h"
#include "gnss/types.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Resampler pass band beyond the IF filter's edge, and its stop band. */
#define RS_PASS_MARGIN_HZ 100e3
#define RS_TRANSITION_HZ  1.6e6
#define RS_ATTEN_DB       80.0
/* AGC update interval (samples at the quantizer's rate) */
#define AGC_CHUNK         256

void fe_cfg_default(fe_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->mode = FE_MODE_DIRECT;
    c->fs_out = FE_FS_CORR_HZ;
    c->fc_in = GNSS_FREQ_L1_HZ;
    c->if_hz = NAN;            /* the frequency plan's, for the mode (fe_init) */
    c->seed = 1;
    c->if_order = 5;
    c->if_bw_hz = 4.2e6;
    c->mag_density = 0.33;
    c->agc_tau_s = 0.01;
    c->decim_mode = DECIM_SUBSAMPLE;
}

static int grow(void **p, size_t *cap, size_t need, size_t elem)
{
    if (need <= *cap) {
        return 0;
    }
    size_t n = *cap ? *cap : 4096;
    while (n < need) {
        n *= 2;
    }
    void *q = realloc(*p, n * elem);
    if (!q) {
        return -1;
    }
    *p = q;
    *cap = n;
    return 0;
}

int fe_init(fe_t *fe, const fe_cfg_t *cfg)
{
    memset(fe, 0, sizeof(*fe));
    fe->cfg = *cfg;
    if (isnan(fe->cfg.if_hz)) {
        /* The frequency plan (fe_format.h): L1 at the ADC's IF before the 27 MS/s stream is
         * decimated, at the correlators' IF in a stream made at their rate, at 0 Hz in a native one. */
        fe->cfg.if_hz = cfg->mode == FE_MODE_ADC27 ? FE_IF_ADC_HZ : (cfg->mode == FE_MODE_NATIVE ? 0.0 : FE_IF_HZ);
    }
    const fe_cfg_t *c = &fe->cfg;
    if (!(c->fs_in > 0.0)) {
        return -1;
    }
    /* L1 to 0 Hz (NATIVE: straight to the IF), plus the carrier-only fix. */
    double f1 = (c->fc_in - GNSS_FREQ_L1_HZ) + c->carrier_fix_hz;
    if (c->mode == FE_MODE_NATIVE) {
        f1 += c->if_hz;
    }
    mix_init(&fe->mix1, f1, c->fs_in, c->start);
    rng_seed(&fe->rng, c->seed);

    if (c->mode == FE_MODE_NATIVE) {
        fe->fs_int = c->fs_in;
        fe->step = 1.0;
        return 0;
    }
    fe->fs_int = (c->mode == FE_MODE_ADC27) ? FE_FS_ADC_HZ : c->fs_out;
    fe->step = c->fs_in / fe->fs_int;

    double lo_rate = c->fs_in < fe->fs_int ? c->fs_in : fe->fs_int;
    double f_pass = 0.5 * c->if_bw_hz + RS_PASS_MARGIN_HZ;
    if (f_pass > 0.42 * lo_rate) {
        f_pass = 0.42 * lo_rate;
    }
    double f_stop = lo_rate - f_pass;
    if (f_stop > f_pass + RS_TRANSITION_HZ) {
        f_stop = f_pass + RS_TRANSITION_HZ;
    }
    if (resamp_init(&fe->rs, c->fs_in, fe->fs_int, f_pass, f_stop, RS_ATTEN_DB, c->start) != 0) {
        return -1;
    }
    if (c->if_order > 0 && iir_butter_lowpass(&fe->lpf, c->if_order, 0.5 * c->if_bw_hz, fe->fs_int) != 0) {
        resamp_free(&fe->rs);
        return -1;
    }
    mix_init(&fe->mix2, c->if_hz, fe->fs_int, 0);
    quant2_init(&fe->q, c->mag_density, c->agc_tau_s * fe->fs_int, AGC_CHUNK);
    decim_init(&fe->dec, c->decim_mode, c->decim_phase);
    return 0;
}

size_t fe_max_out(const fe_t *fe, size_t n)
{
    double r = (fe->cfg.mode == FE_MODE_NATIVE) ? 1.0 : 1.0 / fe->step;
    if (fe->cfg.mode == FE_MODE_ADC27) {
        r /= FE_DECIM;
    }
    return (size_t)ceil((double)n * r) + 64;
}

size_t fe_process(fe_t *fe, float *in, size_t n, float *out_iq, uint8_t *out_codes, size_t max_out)
{
    const fe_cfg_t *c = &fe->cfg;
    if (c->dc_i != 0.0 || c->dc_q != 0.0) {
        float di = (float)c->dc_i, dq = (float)c->dc_q;
        for (size_t k = 0; k < n; k++) {
            in[2 * k] -= di;
            in[2 * k + 1] -= dq;
        }
    }
    mix_apply(&fe->mix1, in, n);
    fe->n_in += (int64_t)n;

    if (c->mode == FE_MODE_NATIVE) {
        size_t m = n < max_out ? n : max_out;
        if (c->noise_sigma > 0.0) {
            rng_add_noise(&fe->rng, in, m, c->noise_sigma);
        }
        memcpy(out_iq, in, sizeof(float) * 2 * m);
        fe->n_out += (int64_t)m;
        return m;
    }

    /* Internal-rate samples this call can produce. */
    size_t cap_int = (size_t)ceil((double)n / fe->step) + 64;
    size_t lim_int = (c->mode == FE_MODE_ADC27) ? max_out * FE_DECIM : max_out;
    if (cap_int > lim_int) {
        cap_int = lim_int;
    }
    if (grow((void **)&fe->work, &fe->work_cap, 2 * cap_int, sizeof(float)) != 0 ||
        grow((void **)&fe->codes, &fe->codes_cap, cap_int, 1) != 0) {
        return 0;
    }
    size_t m = resamp_process(&fe->rs, in, n, fe->work, cap_int);
    if (c->noise_sigma > 0.0) {
        rng_add_noise(&fe->rng, fe->work, m, c->noise_sigma);
    }
    if (c->if_order > 0) {
        iir_apply(&fe->lpf, fe->work, m);
    }
    mix_apply(&fe->mix2, fe->work, m);
    for (size_t k = 0; k < 2 * m; k++) {
        fe->sumsq += (double)fe->work[k] * fe->work[k];
    }
    fe->n_sumsq += 2 * m;

    size_t nout;
    if (c->mode == FE_MODE_DIRECT) {
        quant2_apply(&fe->q, fe->work, m, out_codes);
        nout = m;
    } else {
        quant2_apply(&fe->q, fe->work, m, fe->codes);
        nout = decim_process(&fe->dec, fe->codes, m, out_codes);
    }
    fe->n_out += (int64_t)nout;
    return nout;
}

double fe_out_rate(const fe_t *fe)
{
    if (fe->cfg.mode == FE_MODE_ADC27) {
        return FE_FS_ADC_HZ / FE_DECIM;
    }
    return fe->fs_int;
}

double fe_out_if(const fe_t *fe)
{
    double f = fe->cfg.if_hz;
    if (fe->cfg.mode == FE_MODE_ADC27) {
        double r = FE_FS_ADC_HZ / FE_DECIM;
        f -= r * floor(f / r + 0.5);
    }
    return f;
}

double fe_out_position(const fe_t *fe, int64_t k)
{
    if (fe->cfg.mode == FE_MODE_ADC27) {
        /* SUBSAMPLE keeps sample 4k + phase; SUM4 averages 4k..4k+3. */
        double off = (fe->cfg.decim_mode == DECIM_SUBSAMPLE) ? (double)fe->dec.phase : 1.5;
        return (double)fe->cfg.start + ((double)k * FE_DECIM + off) * fe->step;
    }
    return (double)fe->cfg.start + (double)k * fe->step;
}

double fe_sigma_for_cn0(double cn0_dbhz, double sig_power, double noise_density, double fs_add)
{
    double n0_target = sig_power / pow(10.0, cn0_dbhz / 10.0);
    double n0_add = n0_target - noise_density;
    if (n0_add < 0.0) {
        return -1.0;
    }
    return sqrt(0.5 * n0_add * fs_add);
}

void fe_free(fe_t *fe)
{
    if (fe->cfg.mode != FE_MODE_NATIVE) {
        resamp_free(&fe->rs);
    }
    free(fe->work);
    free(fe->codes);
    memset(fe, 0, sizeof(*fe));
}
