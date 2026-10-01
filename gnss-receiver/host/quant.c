#include "quant.h"

#include "fe_format.h"

#include <math.h>

/*
 * For Gaussian input the density d(t) = P(|x| > t) has slope
 * dd/dln(t) = -2 t phi(t) = -0.48 at the 1/3 point, so a log-threshold step of
 * g * (d - target) per chunk converges with a time constant of about
 * 1 / (0.48 g) chunks.
 */
#define DENSITY_SLOPE 0.4835
/* Threshold for 1/3 magnitude density with Gaussian input, in sigma. */
#define THR_SIGMA_THIRD 0.9674

void quant2_init(quant2_t *q, double target, double tau_samples, size_t chunk)
{
    q->thr = 0.0;
    q->target = target;
    q->chunk = chunk > 0 ? chunk : 1;
    double tau_chunks = tau_samples / (double)q->chunk;
    if (tau_chunks < 1.0) {
        tau_chunks = 1.0;
    }
    q->gain = 1.0 / (DENSITY_SLOPE * tau_chunks);
    q->fill = 0;
    q->mag_count = 0;
    q->sumsq = 0.0;
    q->total_mag = 0;
    q->total_bits = 0;
}

void quant2_apply(quant2_t *q, const float *iq, size_t n, uint8_t *codes)
{
    for (size_t k = 0; k < n; k++) {
        float i = iq[2 * k], s = iq[2 * k + 1];
        if (q->thr <= 0.0) {
            /* Until the first chunk completes, hold samples back from the AGC's
             * decision by measuring power only; the codes use a provisional
             * threshold from the running RMS. */
            q->sumsq += (double)i * i + (double)s * s;
        }
        double thr = q->thr > 0.0 ? q->thr
                                  : THR_SIGMA_THIRD * sqrt(q->sumsq / (2.0 * (double)(q->fill + 1)));
        unsigned c = 0;
        if (i < 0.0f) {
            c |= FE_CODE_I_SIGN;
        }
        if (fabs((double)i) > thr) {
            c |= FE_CODE_I_MAG;
            q->mag_count++;
        }
        if (s < 0.0f) {
            c |= FE_CODE_Q_SIGN;
        }
        if (fabs((double)s) > thr) {
            c |= FE_CODE_Q_MAG;
            q->mag_count++;
        }
        codes[k] = (uint8_t)c;
        if (++q->fill == q->chunk) {
            double d = (double)q->mag_count / (2.0 * (double)q->chunk);
            if (q->thr <= 0.0) {
                q->thr = THR_SIGMA_THIRD * sqrt(q->sumsq / (2.0 * (double)q->chunk));
            } else {
                q->thr *= exp(q->gain * (d - q->target));
            }
            q->total_mag += q->mag_count;
            q->total_bits += 2 * q->chunk;
            q->fill = 0;
            q->mag_count = 0;
        }
    }
}

double quant2_density(const quant2_t *q)
{
    uint64_t bits = q->total_bits + 2 * q->fill;
    return bits ? (double)(q->total_mag + q->mag_count) / (double)bits : 0.0;
}
