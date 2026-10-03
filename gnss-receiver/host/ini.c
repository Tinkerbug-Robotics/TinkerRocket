#include "ini.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int ini_load_string(ini_t *ini, const char *text)
{
    size_t n = strlen(text);
    ini->text = (char *)malloc(n + 1);
    if (!ini->text) {
        return -1;
    }
    memcpy(ini->text, text, n + 1);
    return 0;
}

int ini_load(ini_t *ini, const char *path)
{
    ini->text = NULL;
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    long n = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (n < 0) {
        fclose(fp);
        return -1;
    }
    ini->text = (char *)malloc((size_t)n + 1);
    if (!ini->text) {
        fclose(fp);
        return -1;
    }
    size_t got = fread(ini->text, 1, (size_t)n, fp);
    ini->text[got] = '\0';
    fclose(fp);
    return 0;
}

/* Trims [b, e) in place of surrounding whitespace. */
static void trim(const char **b, const char **e)
{
    while (*b < *e && isspace((unsigned char)**b)) {
        (*b)++;
    }
    while (*e > *b && isspace((unsigned char)(*e)[-1])) {
        (*e)--;
    }
}

static int span_eq(const char *b, const char *e, const char *s)
{
    size_t n = (size_t)(e - b);
    return strlen(s) == n && strncmp(b, s, n) == 0;
}

int ini_get(const ini_t *ini, const char *section, const char *key, char *out, size_t out_size)
{
    if (!ini->text) {
        return -1;
    }
    int in_section = 0;
    const char *p = ini->text;
    while (*p) {
        const char *eol = strchr(p, '\n');
        if (!eol) {
            eol = p + strlen(p);
        }
        const char *b = p, *e = eol;
        trim(&b, &e);
        if (b < e && *b != ';' && *b != '#') {
            if (*b == '[') {
                const char *close = memchr(b, ']', (size_t)(e - b));
                if (close) {
                    const char *sb = b + 1, *se = close;
                    trim(&sb, &se);
                    in_section = span_eq(sb, se, section);
                }
            } else if (in_section) {
                const char *eq = memchr(b, '=', (size_t)(e - b));
                if (eq) {
                    const char *kb = b, *ke = eq;
                    trim(&kb, &ke);
                    if (span_eq(kb, ke, key)) {
                        const char *vb = eq + 1, *ve = e;
                        for (const char *c = vb; c < ve; c++) {
                            if (*c == ';' && c > vb && isspace((unsigned char)c[-1])) {
                                ve = c;
                                break;
                            }
                        }
                        trim(&vb, &ve);
                        size_t n = (size_t)(ve - vb);
                        if (n + 1 > out_size) {
                            return -1;
                        }
                        memcpy(out, vb, n);
                        out[n] = '\0';
                        return 0;
                    }
                }
            }
        }
        p = *eol ? eol + 1 : eol;
    }
    return -1;
}

void ini_free(ini_t *ini)
{
    free(ini->text);
    ini->text = NULL;
}
