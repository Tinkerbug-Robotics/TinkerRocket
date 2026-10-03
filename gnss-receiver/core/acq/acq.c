#include "gnss/acq.h"

#include "gnss/gmath.h"
#include "gnss/sig.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define TWO_PI 6.283185307179586476925286766559

/* Fine stage: code offsets searched around the coarse peak, in steps of FINE_STEP chips. */
#define FINE_STEP  0.0625
#define FINE_NSTEP 25                                   /* +-0.75 chips */
#define FINE_SPAN  (FINE_STEP * ((FINE_NSTEP - 1) / 2))

size_t acq_work_floats(int n_fft, int n_ms)
{
    size_t n = (size_t)n_fft;
    return n                                    /* twiddles */
           + 2 * n * (size_t)n_ms               /* decimated snapshot */
           + 2 * n * (size_t)n_ms * ACQ_SUBBINS /* spectra */
           + 2 * n                              /* code spectrum */
           + 2 * n                              /* product / IFFT */
           + n;                                 /* non-coherent sum */
}

/* A phase rotator exp(j*w*k) run in double and re-anchored every 1024 steps. */
typedef struct {
    double w, c, s, dc, ds;
    uint64_t k;
} rot_t;

static void rot_init(rot_t *r, double w, uint64_t k0)
{
    r->w = w;
    r->dc = cos(w);
    r->ds = sin(w);
    r->k = k0;
    r->c = cos(w * (double)k0);
    r->s = sin(w * (double)k0);
}

static void rot_next(rot_t *r)
{
    r->k++;
    if ((r->k & 1023u) == 0) {
        r->c = cos(r->w * (double)r->k);
        r->s = sin(r->w * (double)r->k);
    } else {
        double c2 = r->c * r->dc - r->s * r->ds;
        r->s = r->s * r->dc + r->c * r->ds;
        r->c = c2;
    }
}

int acq_prepare(acq_t *a, const acq_cfg_t *cfg, const float *iq, size_t n, float *work)
{
    a->cfg = *cfg;
    a->iq = iq;
    a->n = n;
    const int nf = cfg->n_fft, nms = cfg->n_ms;
    double spms_d = cfg->fs / 1000.0;
    uint64_t spms = (uint64_t)llround(spms_d);
    if (nms < 1 || nms > ACQ_MAX_MS || nf > ACQ_MAX_FFT || fabs(spms_d - (double)spms) > 1e-9 ||
        spms < (uint64_t)nf || n < spms * (uint64_t)nms) {
        return -1;
    }
    a->spms = spms;
    float *p = work;
    if (fft_plan_init(&a->plan, nf, p) != 0) {
        return -1;
    }
    p += nf;
    a->dec = p;
    p += 2 * (size_t)nf * nms;
    a->spec = p;
    p += 2 * (size_t)nf * nms * ACQ_SUBBINS;
    a->code_f = p;
    p += 2 * (size_t)nf;
    a->buf = p;
    p += 2 * (size_t)nf;
    a->acc = p;

    /* Mix L1 to 0 Hz and average into n_fft bins per ms: input sample k lands in bin k * n_fft / spms. */
    memset(a->dec, 0, sizeof(float) * 2 * (size_t)nf * nms);
    float *cnt = a->buf;  /* borrowed: samples per bin of one ms */
    rot_t r;
    rot_init(&r, -TWO_PI * cfg->if_hz / cfg->fs, 0);
    for (int b = 0; b < nms; b++) {
        float *d = a->dec + 2 * (size_t)nf * b;
        const float *src = iq + 2 * (size_t)b * spms;
        memset(cnt, 0, sizeof(float) * (size_t)nf);
        for (uint64_t k = 0; k < spms; k++) {
            double re = (double)src[2 * k], im = (double)src[2 * k + 1];
            size_t bin = (size_t)((k * (uint64_t)nf) / spms);
            d[2 * bin] += (float)(re * r.c - im * r.s);
            d[2 * bin + 1] += (float)(re * r.s + im * r.c);
            cnt[bin] += 1.0f;
            rot_next(&r);
        }
        for (int j = 0; j < nf; j++) {
            d[2 * j] /= cnt[j];
            d[2 * j + 1] /= cnt[j];
        }
    }

    /* Spectra of every 1 ms block at every sub-bin offset. */
    const double fs_dec = (double)nf * 1000.0;
    for (int sb = 0; sb < ACQ_SUBBINS; sb++) {
        double f_sub = cfg->dop_center + 1000.0 * (double)sb / ACQ_SUBBINS;
        rot_t rs;
        rot_init(&rs, -TWO_PI * f_sub / fs_dec, 0);  /* referenced to the snapshot start */
        for (int b = 0; b < nms; b++) {
            float *s = a->spec + 2 * (size_t)nf * ((size_t)sb * nms + b);
            const float *src = a->dec + 2 * (size_t)nf * b;
            for (int k = 0; k < nf; k++) {
                double re = (double)src[2 * k], im = (double)src[2 * k + 1];
                s[2 * k] = (float)(re * rs.c - im * rs.s);
                s[2 * k + 1] = (float)(re * rs.s + im * rs.c);
                rot_next(&rs);
            }
            fft_run(&a->plan, s, 0);
        }
    }
    return 0;
}

/* Non-coherent correlation power over all blocks for one Doppler (sub-bin sb, whole-kHz shift j). */
static void correlate(acq_t *a, int sb, int j)
{
    const int nf = a->cfg.n_fft, nms = a->cfg.n_ms;
    memset(a->acc, 0, sizeof(float) * (size_t)nf);
    for (int b = 0; b < nms; b++) {
        const float *s = a->spec + 2 * (size_t)nf * ((size_t)sb * nms + b);
        float *x = a->buf;
        for (int k = 0; k < nf; k++) {
            /* Remove j kHz: the spectrum moves down by j bins. */
            int ks = (k + j) % nf;
            if (ks < 0) {
                ks += nf;
            }
            float sr = s[2 * ks], si = s[2 * ks + 1];
            float cr = a->code_f[2 * k], ci = a->code_f[2 * k + 1];  /* already conjugated */
            x[2 * k] = sr * cr - si * ci;
            x[2 * k + 1] = sr * ci + si * cr;
        }
        fft_run(&a->plan, x, 1);
        for (int k = 0; k < nf; k++) {
            a->acc[k] += x[2 * k] * x[2 * k] + x[2 * k + 1] * x[2 * k + 1];
        }
    }
}

/* Coarse search over the decimated snapshot. */
static void coarse(acq_t *a, const uint8_t *chips, acq_result_t *res)
{
    const int nf = a->cfg.n_fft;
    const double fs_dec = (double)nf * 1000.0;
    for (int k = 0; k < nf; k++) {
        int c = (int)(((uint64_t)k * GPS_CA_LEN) / (uint64_t)nf);
        a->code_f[2 * k] = chips[c] ? -1.0f : 1.0f;
        a->code_f[2 * k + 1] = 0.0f;
    }
    fft_run(&a->plan, a->code_f, 0);
    for (int k = 0; k < nf; k++) {
        a->code_f[2 * k + 1] = -a->code_f[2 * k + 1];
    }
    int jmax = (int)ceil(a->cfg.dop_max / 1000.0);
    double best = -1.0, sum = 0.0;
    long cells = 0;
    int best_sb = 0, best_j = 0, best_k = 0;
    for (int j = -jmax; j <= jmax; j++) {
        for (int sb = 0; sb < ACQ_SUBBINS; sb++) {
            double off = 1000.0 * j + 1000.0 * sb / ACQ_SUBBINS;
            if (fabs(off) > a->cfg.dop_max + 1e-9) {
                continue;
            }
            correlate(a, sb, j);
            int kmax = 0;
            for (int k = 0; k < nf; k++) {
                sum += (double)a->acc[k];
                if (a->acc[k] > a->acc[kmax]) {
                    kmax = k;
                }
            }
            cells += nf;
            if ((double)a->acc[kmax] > best) {
                best = (double)a->acc[kmax];
                best_sb = sb;
                best_j = j;
                best_k = kmax;
            }
        }
    }
    res->metric = (float)(best / (cells ? sum / (double)cells : 1.0));
    res->dop_hz = a->cfg.dop_center + 1000.0 * best_j + 1000.0 * best_sb / ACQ_SUBBINS;
    /* Decimated sample m averages the input over [m, m+1) / fs_dec: centre half a bin in. */
    double t_epoch = ((double)best_k + 0.5) / fs_dec - 0.5 / a->cfg.fs;
    double ph = fmod(-t_epoch * 1.023e6, (double)GPS_CA_LEN);
    res->code_phase = ph < 0.0 ? ph + GPS_CA_LEN : ph;
}

/*
 * 1 ms correlations on the full-rate snapshot with the replica at code phase
 * phase0 (chips at sample 0) and carrier IF + dop: float arithmetic, a 32.32
 * fixed-point code NCO, and a float rotator re-anchored in double every 1024
 * samples (cheap on the P4, whose FPU is single precision). Returns the
 * non-coherent power; prompts (if not NULL) receive each block's complex sum.
 */
static float fine_corr(const acq_t *a, const uint8_t *chips, double phase0, double dop, float *prompts)
{
    const double fs = a->cfg.fs;
    const uint64_t one = (uint64_t)1 << 32, mod = (uint64_t)GPS_CA_LEN << 32;
    const uint64_t step = (uint64_t)llround(1.023e6 * (1.0 + dop / GNSS_FREQ_L1_HZ) / fs * (double)one);
    double p0 = fmod(phase0, (double)GPS_CA_LEN);
    if (p0 < 0.0) {
        p0 += GPS_CA_LEN;
    }
    uint64_t cp = (uint64_t)llround(p0 * (double)one) % mod;
    const double w = -TWO_PI * (a->cfg.if_hz + dop) / fs;
    const float dc = (float)cos(w), ds = (float)sin(w);
    float power = 0.0f, c = 1.0f, sn = 0.0f;
    uint64_t k = 0;
    for (int b = 0; b < a->cfg.n_ms; b++) {
        float si = 0.0f, sq = 0.0f;
        for (uint64_t e = (uint64_t)(b + 1) * a->spms; k < e; k++) {
            if ((k & 1023u) == 0) {
                c = (float)cos(w * (double)k);
                sn = (float)sin(w * (double)k);
            }
            float code = chips[cp >> 32] ? -1.0f : 1.0f;
            float re = a->iq[2 * k], im = a->iq[2 * k + 1];
            si += code * (re * c - im * sn);
            sq += code * (re * sn + im * c);
            float c2 = c * dc - sn * ds;
            sn = sn * dc + c * ds;
            c = c2;
            cp += step;
            if (cp >= mod) {
                cp -= mod;
            }
        }
        power += si * si + sq * sq;
        if (prompts) {
            prompts[2 * b] = si;
            prompts[2 * b + 1] = sq;
        }
    }
    return power;
}

/* Refines code phase (peak of the correlation over +-FINE_SPAN chips) and Doppler (phase progression). */
static void fine(acq_t *a, const uint8_t *chips, acq_result_t *res)
{
    enum { NSTEP = FINE_NSTEP };
    float pw[NSTEP];
    int ib = 0;
    for (int i = 0; i < NSTEP; i++) {
        pw[i] = fine_corr(a, chips, res->code_phase - FINE_SPAN + i * FINE_STEP, res->dop_hz, NULL);
        if (pw[i] > pw[ib]) {
            ib = i;
        }
    }
    double dk = 0.0;
    if (ib > 0 && ib < NSTEP - 1) {
        double den = (double)pw[ib - 1] - 2.0 * (double)pw[ib] + (double)pw[ib + 1];
        if (den < 0.0) {
            dk = 0.5 * ((double)pw[ib - 1] - (double)pw[ib + 1]) / den;
        }
    }
    double ph = fmod(res->code_phase - FINE_SPAN + ((double)ib + dk) * FINE_STEP, (double)GPS_CA_LEN);
    res->code_phase = ph < 0.0 ? ph + GPS_CA_LEN : ph;

    /* Doppler: mean phase step between consecutive 1 ms prompts, folded for data bits. */
    float pr[2 * ACQ_MAX_MS];
    fine_corr(a, chips, res->code_phase, res->dop_hz, pr);
    float cross = 0.0f, dot = 0.0f;
    for (int b = 1; b < a->cfg.n_ms; b++) {
        float cr = pr[2 * b - 2] * pr[2 * b + 1] - pr[2 * b - 1] * pr[2 * b];
        float dt = pr[2 * b - 2] * pr[2 * b] + pr[2 * b - 1] * pr[2 * b + 1];
        if (dt < 0.0f) {  /* a data bit flipped between the two: fold it out */
            cr = -cr;
            dt = -dt;
        }
        cross += cr;
        dot += dt;
    }
    res->dop_hz += (double)gnss_atan2f(cross, dot) / (TWO_PI * 1e-3);
}

void acq_search(acq_t *a, int prn, acq_result_t *res)
{
    memset(res, 0, sizeof(*res));
    res->prn = prn;
    uint8_t chips[GPS_CA_LEN];
    if (gps_ca_code(prn, chips) != 0) {
        return;
    }
    coarse(a, chips, res);
}

void acq_refine(acq_t *a, acq_result_t *res)
{
    uint8_t chips[GPS_CA_LEN];
    if (gps_ca_code(res->prn, chips) != 0) {
        return;
    }
    fine(a, chips, res);
}
