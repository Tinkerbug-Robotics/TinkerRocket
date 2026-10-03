#include "iir.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846264338328

int iir_butter_lowpass(iir_t *f, int order, double fc_hz, double fs_hz)
{
    memset(f, 0, sizeof(*f));
    if (order < 1 || order > 2 * IIR_MAX_SECTIONS || !(fc_hz > 0.0) || !(fc_hz < 0.5 * fs_hz)) {
        return -1;
    }
    double k = tan(PI * fc_hz / fs_hz);  /* pre-warped */
    int npair = order / 2;
    for (int i = 0; i < npair; i++) {
        /* Pole pair i: s^2 + s/Q + 1, Q = 1 / (2 sin((2i+1) pi / (2N))). */
        double q = 1.0 / (2.0 * sin((2.0 * i + 1.0) * PI / (2.0 * order)));
        double norm = 1.0 / (1.0 + k / q + k * k);
        iir_sos_t *s = &f->sec[f->nsec++];
        s->b0 = k * k * norm;
        s->b1 = 2.0 * s->b0;
        s->b2 = s->b0;
        s->a1 = 2.0 * (k * k - 1.0) * norm;
        s->a2 = (1.0 - k / q + k * k) * norm;
    }
    if (order & 1) {
        /* The real pole: s + 1. */
        iir_sos_t *s = &f->sec[f->nsec++];
        s->b0 = k / (1.0 + k);
        s->b1 = s->b0;
        s->b2 = 0.0;
        s->a1 = (k - 1.0) / (k + 1.0);
        s->a2 = 0.0;
    }
    return 0;
}

void iir_apply(iir_t *f, float *iq, size_t n)
{
    for (int j = 0; j < f->nsec; j++) {
        iir_sos_t *s = &f->sec[j];
        double b0 = s->b0, b1 = s->b1, b2 = s->b2, a1 = s->a1, a2 = s->a2;
        double i1 = s->si[0], i2 = s->si[1], q1 = s->sq[0], q2 = s->sq[1];
        for (size_t k = 0; k < n; k++) {
            double x = iq[2 * k];
            double y = b0 * x + i1;
            i1 = b1 * x - a1 * y + i2;
            i2 = b2 * x - a2 * y;
            iq[2 * k] = (float)y;

            x = iq[2 * k + 1];
            y = b0 * x + q1;
            q1 = b1 * x - a1 * y + q2;
            q2 = b2 * x - a2 * y;
            iq[2 * k + 1] = (float)y;
        }
        s->si[0] = i1;
        s->si[1] = i2;
        s->sq[0] = q1;
        s->sq[1] = q2;
    }
}

double iir_mag(const iir_t *f, double f_hz, double fs_hz)
{
    double w = 2.0 * PI * f_hz / fs_hz;
    double mag = 1.0;
    for (int j = 0; j < f->nsec; j++) {
        const iir_sos_t *s = &f->sec[j];
        /* H(e^jw) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2) */
        double nr = s->b0 + s->b1 * cos(w) + s->b2 * cos(2 * w);
        double ni = -s->b1 * sin(w) - s->b2 * sin(2 * w);
        double dr = 1.0 + s->a1 * cos(w) + s->a2 * cos(2 * w);
        double di = -s->a1 * sin(w) - s->a2 * sin(2 * w);
        mag *= sqrt((nr * nr + ni * ni) / (dr * dr + di * di));
    }
    return mag;
}
