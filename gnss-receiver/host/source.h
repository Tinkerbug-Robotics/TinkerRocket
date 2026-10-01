/*
 * A recorded IQ file run through the front-end emulation, delivered as a
 * stream of float I/Q samples at the correlator's rate (2-bit codes become
 * their +-1/+-3 weights). The file's metadata comes from the manifest, then
 * the .TXT sidecar, then the options. Host only.
 */
#ifndef GNSS_HOST_SOURCE_H
#define GNSS_HOST_SOURCE_H

#include <stddef.h>
#include <stdint.h>

#include "fe_emul.h"
#include "iq_file.h"
#include "manifest.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *file, *manifest;
    double fs, fc;
    int have_fs, have_fc;
    double start_s, dur_s;          /* dur <= 0: to the end */
    fe_cfg_t fe;                    /* mode, IF, filter, AGC (rates filled in by src_open) */
    const char *dc_arg, *carrier_arg;
    double cn0, noise_sigma;
    int have_cn0, have_noise;
    double step_t[8], step_cn0[8];  /* --cn0-at: C/N0 from file second step_t on */
    int nsteps;
} src_opts_t;

typedef struct {
    iqf_t f;
    iq_meta_t meta;
    char path[4096];
    fe_t fe;
    int64_t end;
    float *in;
    size_t in_cap;
    float *buf;                     /* native mode: produced, not yet consumed (interleaved) */
    size_t nbuf, buf_cap, head;
    uint8_t *codes;                 /* 2-bit modes: produced codes, not yet consumed */
    size_t ncodes, codes_cap, chead;
    double fs_out, if_out;
    int eof;
    double step_t[8], step_sigma[8];  /* noise from file second step_t on */
    int nsteps, next_step;
} src_t;

void src_default_opts(src_opts_t *o);

/* Takes the option at argv[*i] if it is one of the source's (advancing *i past
 * its value): returns 1 if taken, 0 if not a source option, -1 if malformed. */
int src_parse_opt(src_opts_t *o, int argc, char **argv, int *i);

/* Prints the source options' help. */
void src_usage(void);

int src_open(src_t *s, src_opts_t *o);

/* Reads up to n samples at the output rate; returns how many (fewer only at the end).
 * 2-bit modes deliver their codes' weights (+-1, +-3). */
size_t src_read(src_t *s, float *iq, size_t n);

/* 2-bit modes only: reads up to n sample codes (fe_format.h nibbles). */
size_t src_read_codes(src_t *s, uint8_t *codes, size_t n);

void src_close(src_t *s);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_SOURCE_H */
