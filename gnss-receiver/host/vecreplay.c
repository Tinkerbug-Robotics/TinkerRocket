/*
 * vecreplay: replays a test-vector directory (gnssrx --vectors) through a fresh
 * golden correlator model and checks every dump bit for bit. It is the
 * reference for the HDL testbench, which does the same with the FPGA design:
 *
 *   samples.u2     correlator input, two sample codes per byte, earlier in the high nibble
 *   commands.csv   each command and the sample (t_sample) at which the P4 presents it
 *   dumps.csv      the dumps the correlator must produce, in order per channel
 *
 *   vecreplay DIR
 */
#define _POSIX_C_SOURCE 200809L

#include "corr_model.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint64_t t;
    corr_cmd_t c;
} timed_cmd_t;

static long read_cmds(const char *path, timed_cmd_t **out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    char line[512];
    long n = 0, cap = 256;
    timed_cmd_t *v = (timed_cmd_t *)malloc(sizeof(*v) * (size_t)cap);
    if (!fgets(line, sizeof(line), f)) {  /* header */
        fclose(f);
        free(v);
        return -1;
    }
    while (fgets(line, sizeof(line), f)) {
        unsigned long long t, ts, cp, tap, cw;
        int type, ch, prn, carr;
        if (sscanf(line, "%llu,%d,%d,%d,%llu,%llu,%llu,%d,%llu", &t, &type, &ch, &prn, &ts, &cp, &tap, &carr, &cw) != 9) {
            continue;
        }
        if (n == cap) {
            cap *= 2;
            v = (timed_cmd_t *)realloc(v, sizeof(*v) * (size_t)cap);
        }
        memset(&v[n], 0, sizeof(v[n]));
        v[n].t = t;
        v[n].c.type = (uint8_t)type;
        v[n].c.ch = (uint8_t)ch;
        v[n].c.sig = GNSS_SIG_GPS_L1CA;
        v[n].c.prn = (uint8_t)prn;
        v[n].c.t_start = ts;
        v[n].c.code_phase = cp;
        v[n].c.tap_offset = tap;
        v[n].c.carr_word = carr;
        v[n].c.code_word = cw;
        n++;
    }
    fclose(f);
    *out = v;
    return n;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: vecreplay DIR\n");
        return 2;
    }
    char p[4200];
    snprintf(p, sizeof(p), "%s/samples.u2", argv[1]);
    FILE *fs = fopen(p, "rb");
    snprintf(p, sizeof(p), "%s/dumps.csv", argv[1]);
    FILE *fd = fopen(p, "r");
    snprintf(p, sizeof(p), "%s/commands.csv", argv[1]);
    timed_cmd_t *cmds = NULL;
    long ncmd = read_cmds(p, &cmds);
    if (!fs || !fd || ncmd < 0) {
        fprintf(stderr, "vecreplay: %s needs samples.u2, commands.csv and dumps.csv\n", argv[1]);
        return 1;
    }
    fseek(fs, 0, SEEK_END);
    long nbytes = ftell(fs);
    fseek(fs, 0, SEEK_SET);
    size_t n = (size_t)nbytes * 2;
    uint8_t *raw = (uint8_t *)malloc((size_t)nbytes), *codes = (uint8_t *)malloc(n);
    if (!raw || !codes || fread(raw, 1, (size_t)nbytes, fs) != (size_t)nbytes) {
        return 1;
    }
    for (long k = 0; k < nbytes; k++) {
        codes[2 * k] = raw[k] >> 4;
        codes[2 * k + 1] = raw[k] & 0xF;
    }
    corr_model_t *m = (corr_model_t *)malloc(sizeof(corr_model_t));
    corr_model_cfg_t cfg;
    corr_model_default_cfg(&cfg);
    corr_model_init(m, &cfg);

    /* Run the model, presenting each command at its sample; collect dumps. */
    size_t cap = 1 << 16, ndump = 0;
    corr_dump_t *got = (corr_dump_t *)malloc(sizeof(corr_dump_t) * cap);
    long ci = 0;
    uint64_t t = 0;
    while (t < n) {
        while (ci < ncmd && cmds[ci].t <= t) {
            corr_model_command(m, &cmds[ci].c);
            ci++;
        }
        uint64_t next = ci < ncmd && cmds[ci].t < n ? cmds[ci].t : n;
        if (next <= t) {
            next = t + 1;
        }
        corr_dump_t d[64];
        int nd = corr_model_process(m, t, codes + t, (size_t)(next - t), d, 64);
        for (int k = 0; k < nd; k++) {
            if (ndump == cap) {
                cap *= 2;
                got = (corr_dump_t *)realloc(got, sizeof(corr_dump_t) * cap);
            }
            got[ndump++] = d[k];
        }
        t = next;
    }

    /* Compare with dumps.csv, matched by (channel, seq). */
    char line[512];
    fgets(line, sizeof(line), fd);
    long nwant = 0, bad = 0;
    while (fgets(line, sizeof(line), fd)) {
        unsigned long long ts, cp, cw;
        unsigned seq, ph, cyc;
        int ch, carr;
        double v[6];
        if (sscanf(line, "%d,%u,%llu,%llu,%u,%u,%d,%llu,%lf,%lf,%lf,%lf,%lf,%lf", &ch, &seq, &ts, &cp, &ph, &cyc, &carr,
                   &cw, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 14) {
            continue;
        }
        nwant++;
        const corr_dump_t *g = NULL;
        for (size_t k = 0; k < ndump; k++) {
            if (got[k].ch == ch && got[k].seq == seq && got[k].t_samp == ts) {
                g = &got[k];
                break;
            }
        }
        double gv[6] = {0};
        if (g) {
            gv[0] = g->ie, gv[1] = g->qe, gv[2] = g->ip, gv[3] = g->qp, gv[4] = g->il, gv[5] = g->ql;
        }
        int ok = g && g->code_phase == cp && g->carr_phase == ph && g->carr_cycles == cyc && g->carr_word == carr &&
                 g->code_word == cw;
        for (int k = 0; ok && k < 6; k++) {
            ok = gv[k] == v[k];
        }
        if (!ok) {
            if (bad < 5) {
                fprintf(stderr, "mismatch: ch %d seq %u t %llu%s\n", ch, seq, ts, g ? "" : " (dump missing)");
            }
            bad++;
        }
    }
    printf("vecreplay %s: %zu samples, %ld commands, %ld dumps expected, %ld mismatches\n", argv[1], n, ncmd, nwant,
           bad);
    fclose(fs);
    fclose(fd);
    free(raw);
    free(codes);
    free(m);
    free(got);
    free(cmds);
    return bad || nwant == 0 ? 1 : 0;
}
