/*
 * A minimal INI reader for data/iq_files.ini: [section] headers, key = value
 * lines, comments on lines starting with ';' or '#', and after a ';' that
 * follows whitespace. Host only.
 */
#ifndef GNSS_HOST_INI_H
#define GNSS_HOST_INI_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char *text;  /* the whole file */
} ini_t;

int ini_load(ini_t *ini, const char *path);
int ini_load_string(ini_t *ini, const char *text);

/* Copies the value of key in section into out; returns 0 if found. */
int ini_get(const ini_t *ini, const char *section, const char *key, char *out, size_t out_size);

void ini_free(ini_t *ini);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_INI_H */
