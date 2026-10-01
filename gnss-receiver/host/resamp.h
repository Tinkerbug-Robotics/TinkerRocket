/*
 * Arbitrary-ratio polyphase resampler for complex samples. A Kaiser-windowed
 * sinc prototype is tabulated at RESAMP_PHASES phases per input sample and
 * interpolated linearly between phases, so the ratio can vary with time: the
 * warp hook scales the sample clock by (1 + delta(t)), which is how the
 * oscillator g-sensitivity injection will run (milestone 7). Host only.
 *
 * Output k is the input evaluated at position tau(k) = start + sum of steps,
 * in input samples; with no warp, tau(k) = start + k * fs_in / fs_out.
 */
#ifndef GNSS_HOST_RESAMP_H
#define GNSS_HOST_RESAMP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RESAMP_PHASES 256

/* Fractional sample-clock error delta at output time t_out (s since the start). */
typedef double (*resamp_warp_fn)(void *ctx, double t_out);

typedef struct {
    int ntaps;             /* taps per phase (even) */
    int half;              /* ntaps / 2 */
    float *h;              /* (RESAMP_PHASES + 1) * ntaps, each row time-reversed for a forward dot product */
    double step;           /* input samples per output sample, nominal */
    double fs_out;
    float *bi, *bq;        /* input history, planar */
    size_t cap, len;       /* samples */
    int64_t buf_start;     /* absolute input index of bi[0] */
    int64_t tau_int;       /* position of the next output: integer part */
    double tau_frac;       /*                              fraction [0,1) */
    int64_t out_count;
    resamp_warp_fn warp;
    void *warp_ctx;
    float *taps;           /* scratch */
} resamp_t;

/*
 * Pass band to f_pass, stop band from f_stop (Hz, both below the lower of the
 * two Nyquist frequencies' images), atten_db of stop-band attenuation. The
 * first output sits at input index start (samples before it read as zero).
 */
int resamp_init(resamp_t *r, double fs_in, double fs_out, double f_pass, double f_stop,
                double atten_db, int64_t start);

/*
 * Pushes n interleaved I,Q input samples and writes up to max_out outputs.
 * Outputs that do not fit stay pending and come out on the next call.
 */
size_t resamp_process(resamp_t *r, const float *in, size_t n, float *out, size_t max_out);

/* Input position (in input samples) of the next output. */
double resamp_position(const resamp_t *r);

void resamp_free(resamp_t *r);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_RESAMP_H */
