/*
 * In-place radix-2 complex FFT, float32, our own. Power-of-two sizes only, which
 * is what acquisition uses and what ESP-DSP offers on the P4.
 */
#ifndef GNSS_FFT_H
#define GNSS_FFT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int n;          /* points */
    int log2n;
    float *tw;      /* n/2 twiddles exp(-j*2*pi*k/n), interleaved re, im */
} fft_plan_t;

/* tw_storage holds n floats. Returns -1 unless n is a power of two >= 2. */
int fft_plan_init(fft_plan_t *p, int n, float *tw_storage);

/* x: n interleaved complex values, transformed in place. inverse != 0 gives the
 * unscaled inverse (multiply by 1/n yourself). */
void fft_run(const fft_plan_t *p, float *x, int inverse);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FFT_H */
