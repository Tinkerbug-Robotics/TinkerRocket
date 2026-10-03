#define _POSIX_C_SOURCE 200809L

#include "iq_file.h"

#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

/* Bytes per complex sample, times two (the packed 2-bit format holds two samples a byte). */
static int64_t half_bytes(iqf_format_t fmt)
{
    return fmt == IQF_MAX2769_2B ? 1 : 4;
}

int iqf_open(iqf_t *f, const char *path, iqf_format_t fmt, double fs, double fc)
{
    memset(f, 0, sizeof(*f));
    f->fp = fopen(path, "rb");
    if (!f->fp) {
        return -1;
    }
    if (fseeko(f->fp, 0, SEEK_END) != 0) {
        fclose(f->fp);
        f->fp = NULL;
        return -1;
    }
    off_t bytes = ftello(f->fp);
    fseeko(f->fp, 0, SEEK_SET);
    f->fmt = fmt;
    f->fs = fs;
    f->fc = fc;
    f->nsamp = (int64_t)bytes * 2 / half_bytes(fmt);
    f->pos = 0;
    return 0;
}

int iqf_seek(iqf_t *f, int64_t sample)
{
    if (sample < 0 || sample > f->nsamp) {
        return -1;
    }
    if (fseeko(f->fp, (off_t)(sample * half_bytes(f->fmt) / 2), SEEK_SET) != 0) {
        return -1;
    }
    f->pos = sample;
    return 0;
}

/* The packed MAX2769 format: whole bytes from the one holding sample pos, skipping its first
 * nibble when pos is odd. */
static size_t read_max2769(iqf_t *f, float *iq, size_t n)
{
    const int odd = (int)(f->pos & 1);
    const size_t nbytes = (n + (size_t)odd + 1) / 2;
    if (nbytes > f->raw_cap * 2) {
        int8_t *p = (int8_t *)realloc(f->raw, nbytes);
        if (!p) {
            return 0;
        }
        f->raw = p;
        f->raw_cap = (nbytes + 1) / 2;
    }
    if (fseeko(f->fp, (off_t)(f->pos / 2), SEEK_SET) != 0) {
        return 0;
    }
    const size_t got_bytes = fread(f->raw, 1, nbytes, f->fp);
    size_t got = 0;
    for (size_t b = 0; b < got_bytes && got < n; b++) {
        const unsigned byte = (uint8_t)f->raw[b];
        for (int j = (b == 0 ? odd : 0); j < 2 && got < n; j++) {
            const unsigned nib = (byte >> (4 - 4 * j)) & 0xFu;
            const float i = (nib & 8u) ? 3.0f : 1.0f, q = (nib & 2u) ? 3.0f : 1.0f;
            iq[2 * got] = (nib & 4u) ? -i : i;
            iq[2 * got + 1] = (nib & 1u) ? -q : q;
            got++;
        }
    }
    if (f->pos + (int64_t)got > f->nsamp) {
        got = (size_t)(f->nsamp - f->pos);
    }
    f->pos += (int64_t)got;
    return got;
}

size_t iqf_read(iqf_t *f, float *iq, size_t n)
{
    if (f->fmt == IQF_MAX2769_2B) {
        return read_max2769(f, iq, n);
    }
    if (n > f->raw_cap) {
        int8_t *p = (int8_t *)realloc(f->raw, n * 2);
        if (!p) {
            return 0;
        }
        f->raw = p;
        f->raw_cap = n;
    }
    size_t got = fread(f->raw, 2, n, f->fp);
    for (size_t k = 0; k < 2 * got; k++) {
        iq[k] = (float)f->raw[k];
    }
    f->pos += (int64_t)got;
    return got;
}

void iqf_close(iqf_t *f)
{
    if (f->fp) {
        fclose(f->fp);
    }
    free(f->raw);
    memset(f, 0, sizeof(*f));
}

int iqf_read_sidecar(const char *path, double *fs, double *fc)
{
    size_t len = strlen(path);
    char *side = (char *)malloc(len + 5);
    if (!side) {
        return -1;
    }
    strcpy(side, path);
    char *dot = strrchr(side, '.');
    char *slash = strrchr(side, '/');
    if (dot && (!slash || dot > slash)) {
        strcpy(dot, ".TXT");
    } else {
        strcat(side, ".TXT");
    }
    FILE *fp = fopen(side, "r");
    free(side);
    if (!fp) {
        return -1;
    }
    char line[256];
    int found = 0;
    while (fgets(line, sizeof(line), fp)) {
        double v;
        if (sscanf(line, "sample_rate=%lf", &v) == 1) {
            *fs = v;
            found = 1;
        } else if (sscanf(line, "center_frequency=%lf", &v) == 1) {
            *fc = v;
            found = 1;
        }
    }
    fclose(fp);
    return found ? 0 : -1;
}
