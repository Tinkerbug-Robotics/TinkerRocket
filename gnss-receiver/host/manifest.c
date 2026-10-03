#define _POSIX_C_SOURCE 200809L

#include "manifest.h"

#include "iq_file.h"

#include "ini.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef GNSS_DATA_DIR
#define GNSS_DATA_DIR "data"
#endif

static double get_num(const ini_t *ini, const char *sec, const char *key, double dflt)
{
    char v[64];
    if (ini_get(ini, sec, key, v, sizeof(v)) != 0) {
        return dflt;
    }
    char *end;
    double d = strtod(v, &end);
    return end != v ? d : dflt;
}

static void get_str(const ini_t *ini, const char *sec, const char *key, char *out, size_t n)
{
    if (ini_get(ini, sec, key, out, n) != 0) {
        out[0] = '\0';
    }
}

int manifest_lookup(const char *manifest_path, const char *basename, iq_meta_t *m)
{
    memset(m, 0, sizeof(*m));
    ini_t ini;
    if (ini_load(&ini, manifest_path) != 0) {
        return -1;
    }
    char v[64];
    if (ini_get(&ini, basename, "fs", v, sizeof(v)) != 0) {
        ini_free(&ini);
        return -1;
    }
    m->found = 1;
    m->fs = get_num(&ini, basename, "fs", 0.0);
    m->fc = get_num(&ini, basename, "fc", 0.0);
    char s[64];
    get_str(&ini, basename, "noise", s, sizeof(s));
    m->has_noise = (strcmp(s, "yes") == 0);
    get_str(&ini, basename, "dc", s, sizeof(s));
    if (strcmp(s, "auto") == 0) {
        m->dc_auto = 1;
    } else if (s[0]) {
        if (sscanf(s, "%lf,%lf", &m->dc_i, &m->dc_q) == 1) {
            m->dc_q = m->dc_i;
        }
    }
    m->carrier_fix_hz = get_num(&ini, basename, "carrier_fix_hz", 0.0);
    m->sig_power = get_num(&ini, basename, "sig_power", 0.0);
    m->noise_density = get_num(&ini, basename, "noise_density", 0.0);
    char iono[256];
    if (ini_get(&ini, basename, "iono_params", iono, sizeof(iono)) == 0 &&
        sscanf(iono, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &m->iono[0], &m->iono[1], &m->iono[2], &m->iono[3], &m->iono[4],
               &m->iono[5], &m->iono[6], &m->iono[7]) == 8) {
        m->have_iono = 1;
    }
    get_str(&ini, basename, "tropo", s, sizeof(s));
    m->tropo_none = (strcmp(s, "none") == 0);
    get_str(&ini, basename, "generator", m->generator, sizeof(m->generator));
    get_str(&ini, basename, "start_gpst", m->start_gpst, sizeof(m->start_gpst));
    get_str(&ini, basename, "truth", m->truth, sizeof(m->truth));
    get_str(&ini, basename, "nav", m->nav, sizeof(m->nav));
    get_str(&ini, basename, "format", s, sizeof(s));
    m->format = strcmp(s, "max2769_2bit") == 0 ? IQF_MAX2769_2B : IQF_CS8;
    ini_free(&ini);
    return 0;
}

const char *manifest_default_path(void)
{
    const char *env = getenv("GNSS_MANIFEST");
    if (env && env[0]) {
        return env;
    }
    return GNSS_DATA_DIR "/iq_files.ini";
}

static int file_exists(const char *p)
{
    struct stat st;
    return stat(p, &st) == 0 && S_ISREG(st.st_mode);
}

int manifest_resolve_iq(const char *arg, char *out, unsigned long out_size)
{
    if (file_exists(arg)) {
        snprintf(out, out_size, "%s", arg);
        return 0;
    }
    const char *dir = getenv("GNSS_IQ_DIR");
    if (dir && dir[0] && !strchr(arg, '/')) {
        snprintf(out, out_size, "%s/%s", dir, arg);
        if (file_exists(out)) {
            return 0;
        }
    }
    snprintf(out, out_size, "%s", arg);
    return -1;
}
