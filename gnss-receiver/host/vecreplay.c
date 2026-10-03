/*
 * vecreplay: replays a test-vector directory (gnssrx --vectors) through a fresh
 * golden correlator model and checks every dump bit for bit. It is the
 * reference for the HDL testbench, which does the same with the FPGA design:
 *
 *   samples.u2     correlator input, two sample codes per byte, earlier in the high nibble
 *   commands.csv   each command and the sample (t_sample) at which the P4 presents it,
 *                  with the START's signal and second tap offset in the last two columns
 *   dumps.csv      the dumps the correlator must produce, in order per channel: the six
 *                  I/Q pairs are early, prompt, late, very early, very late and data
 * and, when the run had the notch stage (gnssrx --mitig notch), its own vectors, checked first
 * against a fresh fpga/model/notch.c:
 *   notch_in.u2       the stage's input codes, packed as samples.u2
 *   notch_out.u2      its output codes (the correlators' input: samples.u2 again)
 *   notch_power.csv   each millisecond: the power counters as the P4 reads them, the requantizer's
 *                     threshold, and each notch's zacc
 *
 *   vecreplay DIR
 */
#define _POSIX_C_SOURCE 200809L

#include "corr_model.h"
#include "notch.h"

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
        unsigned long long t, ts, cp, tap, cw, tap2;
        unsigned tag;
        int type, ch, prn, carr, sig;
        if (sscanf(line, "%llu,%d,%d,%d,%llu,%llu,%llu,%d,%llu,%u,%d,%llu", &t, &type, &ch, &prn, &ts, &cp, &tap, &carr,
                   &cw, &tag, &sig, &tap2) != 12) {
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
        v[n].c.sig = (uint8_t)sig;
        v[n].c.tap_offset2 = tap2;
        v[n].c.prn = (uint8_t)prn;
        v[n].c.t_start = ts;
        v[n].c.code_phase = cp;
        v[n].c.tap_offset = tap;
        v[n].c.carr_word = carr;
        v[n].c.code_word = cw;
        v[n].c.apply_seq = tag;
        n++;
    }
    fclose(f);
    *out = v;
    return n;
}

static uint8_t *read_u2(const char *path, size_t *n)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    const long nb = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *raw = (uint8_t *)malloc((size_t)nb), *c = (uint8_t *)malloc((size_t)nb * 2);
    if (!raw || !c || fread(raw, 1, (size_t)nb, f) != (size_t)nb) {
        fclose(f);
        free(raw);
        free(c);
        return NULL;
    }
    fclose(f);
    for (long k = 0; k < nb; k++) {
        c[2 * k] = raw[k] >> 4;
        c[2 * k + 1] = raw[k] & 0xF;
    }
    free(raw);
    *n = (size_t)nb * 2;
    return c;
}

/* The notch stage's vectors, if the directory has them: -1 if not, else the mismatches. The
 * counters latch every ms_samples (notch.ini; 6750 by default, 1 ms at 6.75 MS/s). */
static long replay_notch(const char *dir)
{
    char p[4200], line[1024];
    size_t n_in = 0, n_out = 0;
    snprintf(p, sizeof(p), "%s/notch_in.u2", dir);
    uint8_t *in = read_u2(p, &n_in);
    if (!in) {
        return -1;
    }
    snprintf(p, sizeof(p), "%s/notch_out.u2", dir);
    uint8_t *out = read_u2(p, &n_out);
    size_t ms_samples = 6750;
    snprintf(p, sizeof(p), "%s/notch.ini", dir);
    FILE *fi = fopen(p, "r");
    if (fi) {
        while (fgets(line, sizeof(line), fi)) {
            unsigned long v;
            if (sscanf(line, "ms_samples = %lu", &v) == 1 && v > 0) {
                ms_samples = (size_t)v;
            }
        }
        fclose(fi);
    }
    snprintf(p, sizeof(p), "%s/notch_power.csv", dir);
    FILE *fp = fopen(p, "r");
    if (!out || !fp || n_out != n_in || !fgets(line, sizeof(line), fp)) {
        fprintf(stderr, "vecreplay: %s: notch vectors incomplete\n", dir);
        free(in);
        free(out);
        if (fp) {
            fclose(fp);
        }
        return 1;
    }
    notch_t nt;
    notch_init(&nt);
    long rows = 0, bad = 0;
    for (size_t k = 0; k < n_in; k++) {
        const uint8_t o = notch_sample(&nt, in[k]);
        if (o != out[k]) {
            if (bad < 5) {
                fprintf(stderr, "notch mismatch: sample %zu: model %x, vector %x\n", k, o, out[k]);
            }
            bad++;
        }
        if ((k + 1) % ms_samples != 0) {
            continue;
        }
        unsigned long long ms, pin, pout;
        long long t, z[2 * NOTCH_N];
        if (!fgets(line, sizeof(line), fp) ||
            sscanf(line, "%llu,%llu,%llu,%lld,%lld,%lld,%lld,%lld", &ms, &pin, &pout, &t, &z[0], &z[1], &z[2], &z[3]) <
                4 + 2 * NOTCH_N) {
            if (bad < 5) {
                fprintf(stderr, "notch: counter row missing at sample %zu\n", k + 1);
            }
            bad++;
            continue;
        }
        uint64_t mi, mo;
        notch_take_power(&nt, &mi, &mo);
        int ok = mi == pin && mo == pout && nt.t == t;
        for (int s2 = 0; ok && s2 < NOTCH_N; s2++) {
            ok = nt.s[s2].zr == z[2 * s2] && nt.s[s2].zi == z[2 * s2 + 1];
        }
        if (!ok) {
            if (bad < 5) {
                fprintf(stderr, "notch mismatch: ms %llu: counters or state\n", ms);
            }
            bad++;
        }
        rows++;
    }
    fclose(fp);
    printf("vecreplay %s: notch stage, %zu samples, %ld ms of counters, %ld mismatches\n", dir, n_in, rows, bad);
    free(in);
    free(out);
    return bad;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: vecreplay DIR\n");
        return 2;
    }
    const long bad_notch = replay_notch(argv[1]);
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
        unsigned seq, ph, cyc, flags;
        int ch, carr;
        double v[12];
        if (sscanf(line, "%d,%u,%llu,%llu,%u,%u,%d,%llu,%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &ch, &seq,
                   &ts, &cp, &ph, &cyc, &carr, &cw, &flags, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
                   &v[8], &v[9], &v[10], &v[11]) != 21) {
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
        double gv[12] = {0};
        if (g) {
            gv[0] = g->ie, gv[1] = g->qe, gv[2] = g->ip, gv[3] = g->qp, gv[4] = g->il, gv[5] = g->ql;
            gv[6] = g->ive, gv[7] = g->qve, gv[8] = g->ivl, gv[9] = g->qvl, gv[10] = g->id, gv[11] = g->qd;
        }
        int ok = g && g->code_phase == cp && g->carr_phase == ph && g->carr_cycles == cyc && g->carr_word == carr &&
                 g->code_word == cw && g->flags == flags;
        for (int k = 0; ok && k < 12; k++) {
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
    return bad || nwant == 0 || bad_notch > 0 ? 1 : 0;
}
