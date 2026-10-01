/* Recorded IQ files: reading, seeking, sidecars. Host only. */
#ifndef GNSS_HOST_IQ_FILE_H
#define GNSS_HOST_IQ_FILE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IQF_CS8 = 0,     /* int8 I, int8 Q interleaved, sample = I + jQ (HackRF, gps-sdr-sim, SignalSim .C8) */
    IQF_MAX2769_2B   /* MAX2769 2-bit sign/magnitude I and Q, two samples a byte, the older in the high
                        nibble, each nibble I-mag, I-sign, Q-mag, Q-sign from the top (PSAS's jGPS);
                        read as levels +-1 and +-3 */
} iqf_format_t;

typedef struct {
    FILE *fp;
    iqf_format_t fmt;
    double fs;        /* sample rate, Hz */
    double fc;        /* centre frequency, Hz */
    int64_t nsamp;    /* complex samples in the file */
    int64_t pos;      /* index of the next sample to be read */
    int8_t *raw;
    size_t raw_cap;   /* complex samples raw holds */
} iqf_t;

int iqf_open(iqf_t *f, const char *path, iqf_format_t fmt, double fs, double fc);
int iqf_seek(iqf_t *f, int64_t sample);

/* Reads up to n samples as interleaved float I,Q; returns the number read (0 at the end). */
size_t iqf_read(iqf_t *f, float *iq, size_t n);

void iqf_close(iqf_t *f);

/*
 * Reads the Mayhem-style sidecar beside path (same name, extension .TXT:
 * center_frequency=, sample_rate=). Returns 0 and fills whichever it finds, or
 * -1 if there is no sidecar.
 */
int iqf_read_sidecar(const char *path, double *fs, double *fc);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_IQ_FILE_H */
