/*
 * hackrf_tx_ram -- transmit an IQ8 file like `hackrf_transfer -t`, but never read the disk inside the USB loop.
 *
 * hackrf_transfer fills each USB transfer with an fread() of the file from inside libhackrf's transfer
 * callback; at 18.48 Msps the HackRF's own buffer holds ~0.6 ms, so any read that waits on the disk (or on
 * anything else) is an underrun, and each underrun shifts the replayed signal in time. Here a reader thread keeps
 * up to RING_MIB (default 2048) of the file in memory ahead of playback, reading straight from the SSD
 * (F_NOCACHE), and the callback only copies from memory. The process asks for the user-interactive QoS class
 * before libhackrf starts its transfer thread, so that thread inherits it.
 *
 * Options mirror hackrf_transfer's TX subset, and -B prints the same one line per second, so a runner that
 * starts hackrf_transfer can start this instead:
 *   -t FILE  -f FREQ_HZ  -s RATE_HZ  -a AMP(0/1)  -x TXVGA_DB  [-b BBF_HZ]  [-B]
 * Extra:  -D  dry run: no radio, a paced consumer at the sample rate (checks the ring keeps up)
 *         -X  dump: write the bytes the callback would send to stdout, unpaced (checks the content)
 *         -N DB  lower the file's C/N0 by DB: the reader adds white Gaussian noise to every sample before it
 *             enters the ring and scales the sum back to the file's own RMS, y = a x + c n with
 *             a = 10^(-DB/20) and c^2 = a^2 s^2 (10^(DB/10) - 1), s = the file's RMS per rail measured on the
 *             first chunk. A file whose power is its own noise (SignalSim) keeps its level and noise density;
 *             every signal in it drops by DB. n comes from a 32 Mi-entry N(0,1) table read from a random start
 *             per chunk (xoshiro256**, NOISE_SEED, default 1).
 *         -C HZ  shift every carrier by HZ (the rig's carrier correction, -23 on the wide files) with the code
 *             untouched: each sample is rotated by exp(j 2 pi HZ n / fs), n its index in the file, phase continuous
 *             -- the same operation as signalsim/carrier_shift.py, done as the file is read (needs -s). Applied
 *             before -N.
 *         -K  keep the reader thread waking every 2 ms after it stops reading (at the end of the file, or once -L
 *             has filled the ring), as it does while it reads. Underrun test 2026-10-01: underruns bunched up after
 *             the reader's EOF, when the process goes quiet.
 *         -L SECONDS  loop test: fill the ring once from the start of the file, then transmit that RING_MIB over
 *             and over for SECONDS, with no disk reads while transmitting (a long version of the after-EOF state).
 * Env:    RING_MIB (2048), PREFILL_MIB (512), NOISE_SEED (1)
 *
 * Build:  clang -O2 -Wall -o hackrf_tx_ram hackrf_tx_ram.c -I/opt/homebrew/include/libhackrf \
 *             -L/opt/homebrew/lib -lhackrf -lpthread
 */
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <pthread/qos.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <hackrf.h>

#define CHUNK (8u << 20)                       /* reader's read size */

static uint8_t *ring;
static size_t ring_size;
static int fd = -1;
static uint64_t file_size;
static _Atomic uint64_t wpos;                  /* bytes the reader has put in the ring */
static _Atomic uint64_t rpos;                  /* bytes the callback has taken */
static _Atomic int reader_eof;
static _Atomic int stop_req;
static _Atomic int flushed;
static _Atomic uint64_t host_short;            /* callbacks that found the ring short (should stay 0) */
static _Atomic uint64_t sent_bytes;
static _Atomic uint64_t sumsq_q;               /* sum of squares for the power line (reset each second) */
static _Atomic uint64_t sumsq_n;
static _Atomic uint64_t max_cb_ns;
static int keep_tick;                          /* -K */
static double loop_secs;                       /* -L */
static _Atomic uint64_t loop_len;              /* -L: bytes in the ring being replayed, set once it is filled */

static double now_s(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

/* ---- -N: added noise ---- */
#define NTAB (1u << 25)
static double noise_db = 0.0;
static float *ntab;
static float na = 1.0f, nc = 0.0f;             /* y = na x + nc n */
static double file_rms = 0.0;
static uint64_t noise_clipped;                 /* reader thread only */
static uint64_t rs[4];

static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

static uint64_t rnd(void) {                    /* xoshiro256** */
    uint64_t r = rotl(rs[1] * 5, 7) * 9, t = rs[1] << 17;
    rs[2] ^= rs[0];
    rs[3] ^= rs[1];
    rs[1] ^= rs[2];
    rs[0] ^= rs[3];
    rs[2] ^= t;
    rs[3] = rotl(rs[3], 45);
    return r;
}

static void make_noise_table(uint64_t seed) {
    uint64_t z = seed;
    for (int i = 0; i < 4; i++) {              /* splitmix64 seeding */
        z += 0x9e3779b97f4a7c15ULL;
        uint64_t x = z;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        rs[i] = x ^ (x >> 31);
    }
    ntab = malloc(NTAB * sizeof(float));
    for (size_t i = 0; i < NTAB; i += 2) {     /* Box-Muller */
        double u1 = ((rnd() >> 11) + 1.0) * 0x1.0p-53, u2 = (rnd() >> 11) * 0x1.0p-53;
        double r = sqrt(-2.0 * log(u1));
        ntab[i] = (float)(r * cos(2.0 * M_PI * u2));
        ntab[i + 1] = (float)(r * sin(2.0 * M_PI * u2));
    }
}

/* ---- -C: carrier shift ---- */
static double carrier_hz = 0.0, sample_rate = 0.0;

static inline int8_t clip8(float v) {
    long q = lrintf(v);
    if (q > 127) {
        noise_clipped++;
        return 127;
    }
    if (q < -128) {
        noise_clipped++;
        return -128;
    }
    return (int8_t)q;
}

/* Rotate (-C) and/or add noise (-N) to n bytes of interleaved I/Q whose first complex sample is the file's n0-th. */
static void process_chunk(uint8_t *p, size_t n, uint64_t n0) {
    int8_t *s = (int8_t *)p;
    if (noise_db > 0.0 && file_rms == 0.0) {   /* first chunk: the file's own RMS per rail */
        double acc = 0.0;
        for (size_t i = 0; i < n; i++)
            acc += (double)s[i] * s[i];
        file_rms = sqrt(acc / (double)n);
        double a = pow(10.0, -noise_db / 20.0);
        na = (float)a;
        nc = (float)(a * file_rms * sqrt(pow(10.0, noise_db / 10.0) - 1.0));
    }
    size_t j = noise_db > 0.0 ? (size_t)(rnd() & (NTAB - 1)) : 0;
    double step = carrier_hz != 0.0 ? carrier_hz / sample_rate : 0.0;   /* cycles per sample */
    size_t m = n / 2;
    for (size_t k0 = 0; k0 < m; k0 += 4096) {  /* exact phase every 4096 samples, a rotator in between */
        size_t k1 = k0 + 4096 < m ? k0 + 4096 : m;
        double cyc = fmod((double)(n0 + k0) * step, 1.0);
        double c = cos(2.0 * M_PI * cyc), sn = sin(2.0 * M_PI * cyc);
        double wc = cos(2.0 * M_PI * step), ws = sin(2.0 * M_PI * step);
        for (size_t k = k0; k < k1; k++) {
            float i = s[2 * k], q = s[2 * k + 1];
            float ri = i, rq = q;
            if (step != 0.0) {
                ri = (float)(i * c - q * sn);
                rq = (float)(i * sn + q * c);
                double c2 = c * wc - sn * ws;
                sn = c * ws + sn * wc;
                c = c2;
            }
            if (noise_db > 0.0) {
                ri = na * ri + nc * ntab[j];
                rq = na * rq + nc * ntab[(j + 1) & (NTAB - 1)];
                j = (j + 2) & (NTAB - 1);
            }
            s[2 * k] = clip8(ri);
            s[2 * k + 1] = clip8(rq);
        }
    }
}

static void *reader(void *arg) {
    (void)arg;
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INITIATED, 0);
    for (;;) {
        if (atomic_load(&stop_req))
            break;
        uint64_t w = atomic_load_explicit(&wpos, memory_order_relaxed);
        uint64_t r = atomic_load_explicit(&rpos, memory_order_acquire);
        if (loop_secs > 0.0 && w + CHUNK > ring_size) {      /* -L: filled once; it is replayed from here on */
            atomic_store(&loop_len, w);
            break;
        }
        uint64_t space = ring_size - (w - r);
        if (space < CHUNK) {
            usleep(2000);
            continue;
        }
        size_t off = (size_t)(w % ring_size);
        size_t len = CHUNK;
        if (off + len > ring_size)
            len = ring_size - off;
        ssize_t got = read(fd, ring + off, len);
        if (got < 0 && errno == EINTR)
            continue;
        if (got <= 0) {
            if (loop_secs > 0.0)
                atomic_store(&loop_len, w);
            break;
        }
        if (noise_db > 0.0 || carrier_hz != 0.0)
            process_chunk(ring + off, (size_t)got, w / 2);   /* before the callback can see these bytes */
        atomic_store_explicit(&wpos, w + (uint64_t)got, memory_order_release);
    }
    atomic_store(&reader_eof, 1);
    if (keep_tick)                                           /* -K: go on waking every 2 ms, as while reading */
        while (!atomic_load(&stop_req))
            usleep(2000);
    return NULL;
}

/* Take up to `need` bytes from the ring into dst; returns the number taken. Never waits. */
static size_t take(uint8_t *dst, size_t need) {
    uint64_t r = atomic_load_explicit(&rpos, memory_order_relaxed);
    uint64_t lp = atomic_load(&loop_len);
    if (lp) {                                      /* -L: replay the filled ring, endlessly */
        size_t done = 0;
        while (done < need) {
            size_t off = (size_t)((r + done) % lp);
            size_t n = need - done;
            if (off + n > lp)
                n = (size_t)lp - off;
            memcpy(dst + done, ring + off, n);
            done += n;
        }
        atomic_store_explicit(&rpos, r + need, memory_order_release);
        return need;
    }
    uint64_t w = atomic_load_explicit(&wpos, memory_order_acquire);
    size_t avail = (size_t)(w - r);
    size_t n = need < avail ? need : avail;
    size_t off = (size_t)(r % ring_size);
    size_t first = n;
    if (off + first > ring_size)
        first = ring_size - off;
    memcpy(dst, ring + off, first);
    if (n > first)
        memcpy(dst + first, ring, n - first);
    atomic_store_explicit(&rpos, r + n, memory_order_release);
    return n;
}

static void account(const uint8_t *buf, size_t n) {
    const int8_t *s = (const int8_t *)buf;
    uint64_t acc = 0;
    for (size_t i = 0; i < n; i += 64)      /* every 64th byte is plenty for a power estimate */
        acc += (uint64_t)((int)s[i] * (int)s[i]);
    atomic_fetch_add(&sumsq_q, acc);
    atomic_fetch_add(&sumsq_n, (n + 63) / 64);
    atomic_fetch_add(&sent_bytes, n);
}

static int tx_callback(hackrf_transfer *t) {
    struct timeval a, b;
    gettimeofday(&a, NULL);
    if (atomic_load(&stop_req)) {
        t->valid_length = 0;
        return -1;
    }
    size_t need = (size_t)t->buffer_length;
    size_t n = take(t->buffer, need);
    if (n < need) {
        int done = atomic_load(&reader_eof) &&
                   atomic_load(&rpos) >= atomic_load(&wpos);
        if (!done) {                             /* the reader fell behind: send silence rather than wait */
            memset(t->buffer + n, 0, need - n);
            n = need;
            atomic_fetch_add(&host_short, 1);
        }
    }
    account(t->buffer, n);
    t->valid_length = (int)n;
    gettimeofday(&b, NULL);
    uint64_t ns = (uint64_t)((b.tv_sec - a.tv_sec) * 1000000000LL + (b.tv_usec - a.tv_usec) * 1000LL);
    uint64_t m = atomic_load(&max_cb_ns);
    while (ns > m && !atomic_compare_exchange_weak(&max_cb_ns, &m, ns))
        ;
    return n == 0 ? -1 : 0;                      /* nothing left: stop, the flush callback follows */
}

static void flush_callback(void *ctx, int success) {
    (void)ctx;
    (void)success;
    atomic_store(&flushed, 1);
}

static void on_signal(int sig) {
    (void)sig;
    atomic_store(&stop_req, 1);
}

static void print_stats(double dt, uint64_t bytes, hackrf_device *dev) {
    uint64_t q = atomic_exchange(&sumsq_q, 0), n = atomic_exchange(&sumsq_n, 0);
    double pwr = (n && q) ? 10.0 * log10((double)q / (double)n * 2.0 / (127.0 * 127.0)) : -99.0;
    int fill = 0;
    unsigned shortfalls = 0, longest = 0;
    if (dev) {
        hackrf_m0_state st;
        if (hackrf_get_m0_state(dev, &st) == HACKRF_SUCCESS) {
            fill = (int)(st.m4_count - st.m0_count);
            shortfalls = st.num_shortfalls;
            longest = st.longest_shortfall;
        }
    }
    uint64_t lp = atomic_load(&loop_len);
    uint64_t ahead = lp ? lp : atomic_load(&wpos) - atomic_load(&rpos);      /* -L: the loop's length */
    fprintf(stderr, "%.1f MB / %.3f sec = %.1f MB/second, average power %.1f dBfs, %d bytes filled in buffer, "
            "%u underruns, longest %u bytes, ring %llu MiB ahead, host shortfalls %llu, slowest callback %.2f ms\n",
            bytes / 1e6, dt, bytes / 1e6 / dt, pwr, fill, shortfalls, longest,
            (unsigned long long)(ahead >> 20), (unsigned long long)atomic_load(&host_short),
            atomic_exchange(&max_cb_ns, 0) / 1e6);
}

int main(int argc, char **argv) {
    const char *path = NULL;
    uint64_t freq = 0;
    double rate = 10e6;
    int amp = 0, txvga = 0, stats = 0, dry = 0, dump = 0;
    uint32_t bbf = 0;
    int opt;
    int rate_given = 0;
    while ((opt = getopt(argc, argv, "t:f:s:a:x:b:BDXN:C:KL:")) != -1) {
        switch (opt) {
        case 't': path = optarg; break;
        case 'f': freq = strtoull(optarg, NULL, 10); break;
        case 's': rate = atof(optarg); rate_given = 1; break;
        case 'a': amp = atoi(optarg); break;
        case 'x': txvga = atoi(optarg); break;
        case 'b': bbf = (uint32_t)strtoul(optarg, NULL, 10); break;
        case 'B': stats = 1; break;
        case 'D': dry = 1; break;
        case 'X': dump = 1; break;
        case 'N': noise_db = atof(optarg); break;
        case 'C': carrier_hz = atof(optarg); break;
        case 'K': keep_tick = 1; break;
        case 'L': loop_secs = atof(optarg); break;
        default:
            fprintf(stderr, "usage: %s -t FILE -f FREQ -s RATE -a AMP -x TXVGA [-b BBF] [-B] [-D|-X] [-N DB] [-C HZ] "
                    "[-K] [-L SECONDS]\n", argv[0]);
            return 2;
        }
    }
    if (loop_secs < 0.0 || (loop_secs > 0.0 && (dump || noise_db > 0.0 || carrier_hz != 0.0))) {
        fprintf(stderr, "hackrf_tx_ram: -L takes SECONDS > 0, and not with -X, -N or -C\n");
        return 2;
    }
    if (noise_db < 0.0 || noise_db > 30.0) {
        fprintf(stderr, "hackrf_tx_ram: -N takes 0 to 30 dB\n");
        return 2;
    }
    if (carrier_hz != 0.0 && (!rate_given || fabs(carrier_hz) > 1000.0)) {
        fprintf(stderr, "hackrf_tx_ram: -C needs -s, and takes at most +-1000 Hz\n");
        return 2;
    }
    sample_rate = rate;
    if (noise_db > 0.0)
        make_noise_table(getenv("NOISE_SEED") ? strtoull(getenv("NOISE_SEED"), NULL, 10) : 1);
    if (!path || strcmp(path, "-") == 0) {
        fprintf(stderr, "hackrf_tx_ram: -t FILE required (stdin is not supported)\n");
        return 2;
    }
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
    ring_size = (size_t)(getenv("RING_MIB") ? atol(getenv("RING_MIB")) : 2048) << 20;
    size_t prefill = (size_t)(getenv("PREFILL_MIB") ? atol(getenv("PREFILL_MIB")) : 512) << 20;
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "hackrf_tx_ram: cannot open %s: %s\n", path, strerror(errno));
        return 1;
    }
    struct stat sb;
    fstat(fd, &sb);
    file_size = (uint64_t)sb.st_size;
    fcntl(fd, F_NOCACHE, 1);
    if (prefill > file_size)
        prefill = (size_t)file_size;
    if (prefill > ring_size - CHUNK)
        prefill = ring_size - CHUNK;
    ring = mmap(NULL, ring_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (ring == MAP_FAILED) {
        fprintf(stderr, "hackrf_tx_ram: cannot map %zu MiB\n", ring_size >> 20);
        return 1;
    }
    int locked = mlock(ring, ring_size) == 0;
    memset(ring, 0, ring_size);                    /* fault every page in now, not in the callback */
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);
    pthread_t rt;
    pthread_create(&rt, NULL, reader, NULL);
    double t0 = now_s();
    while ((loop_secs > 0.0 || atomic_load(&wpos) < prefill) && !atomic_load(&reader_eof))
        usleep(1000);                              /* -L: the whole ring before the radio starts */
    fprintf(stderr, "hackrf_tx_ram: %s, %.1f GB; ring %zu MiB (%s), %zu MiB read ahead in %.2f s\n", path,
            file_size / 1e9, ring_size >> 20, locked ? "locked" : "not locked", (size_t)(atomic_load(&wpos) >> 20),
            now_s() - t0);
    if (noise_db > 0.0)
        fprintf(stderr, "hackrf_tx_ram: added noise -N %.2f dB: file RMS %.2f LSB per rail, y = %.4f x + %.3f n\n",
                noise_db, file_rms, na, nc);
    if (carrier_hz != 0.0)
        fprintf(stderr, "hackrf_tx_ram: carrier shift -C %+.3f Hz at %.0f Hz (code untouched)\n", carrier_hz,
                sample_rate);
    if (loop_secs > 0.0)
        fprintf(stderr, "hackrf_tx_ram: loop test -L %.0f s over the first %llu MiB of the file, no disk reads while "
                "transmitting\n", loop_secs, (unsigned long long)(atomic_load(&loop_len) >> 20));
    if (keep_tick)
        fprintf(stderr, "hackrf_tx_ram: -K reader keeps waking every 2 ms after it stops reading\n");

    if (dump) {                                    /* content check: the callback's bytes, unpaced */
        uint8_t *buf = malloc(262144);
        for (;;) {
            size_t n = take(buf, 262144);
            if (n == 0) {
                if (atomic_load(&reader_eof) && atomic_load(&rpos) >= atomic_load(&wpos))
                    break;
                usleep(500);
                continue;
            }
            if (fwrite(buf, 1, n, stdout) != n)
                break;
        }
        atomic_store(&stop_req, 1);
        pthread_join(rt, NULL);
        return 0;
    }

    hackrf_device *dev = NULL;
    if (!dry) {
        int r = hackrf_init();
        if (r == HACKRF_SUCCESS)
            r = hackrf_open(&dev);
        if (r != HACKRF_SUCCESS) {
            fprintf(stderr, "hackrf_open() failed: %s (%d)\n", hackrf_error_name(r), r);
            return 1;
        }
        fprintf(stderr, "call hackrf_set_sample_rate(%.0f Hz/%.3f MHz)\n", rate, rate / 1e6);
        hackrf_set_sample_rate(dev, rate);
        if (bbf) {
            fprintf(stderr, "call hackrf_set_baseband_filter_bandwidth(%u Hz/%.3f MHz)\n", bbf, bbf / 1e6);
            hackrf_set_baseband_filter_bandwidth(dev, bbf);
        }
        fprintf(stderr, "call hackrf_set_hw_sync_mode(0)\n");
        hackrf_set_hw_sync_mode(dev, 0);
        fprintf(stderr, "call hackrf_set_freq(%llu Hz/%.3f MHz)\n", (unsigned long long)freq, freq / 1e6);
        hackrf_set_freq(dev, freq);
        fprintf(stderr, "call hackrf_set_amp_enable(%d)\n", amp);
        hackrf_set_amp_enable(dev, (uint8_t)amp);
        hackrf_set_txvga_gain(dev, (uint32_t)txvga);
        hackrf_enable_tx_flush(dev, flush_callback, NULL);
        r = hackrf_start_tx(dev, tx_callback, NULL);
        if (r != HACKRF_SUCCESS) {
            fprintf(stderr, "hackrf_start_tx() failed: %s (%d)\n", hackrf_error_name(r), r);
            hackrf_close(dev);
            hackrf_exit();
            return 1;
        }
        fprintf(stderr, "Stop with Ctrl-C\n");
    }

    /* dry run: a paced consumer in place of the USB loop */
    uint8_t *dbuf = dry ? malloc(262144) : NULL;
    double tick = 262144.0 / (2.0 * rate), next = now_s();
    double last = now_s(), t_tx = now_s();
    uint64_t last_bytes = 0;
    for (;;) {
        if (dry) {
            double t = now_s();
            if (t < next) {
                usleep((useconds_t)((next - t) * 1e6));
                continue;
            }
            next += tick;
            hackrf_transfer tr = {.device = NULL, .buffer = dbuf, .buffer_length = 262144, .valid_length = 0};
            if (tx_callback(&tr) != 0)
                atomic_store(&flushed, 1);
        } else {
            usleep(20000);
        }
        double t = now_s();
        if (t - last >= 1.0) {
            uint64_t b = atomic_load(&sent_bytes);
            if (stats)
                print_stats(t - last, b - last_bytes, dev);
            last = t;
            last_bytes = b;
        }
        if (atomic_load(&flushed) || atomic_load(&stop_req))
            break;
        if (loop_secs > 0.0 && t - t_tx >= loop_secs)
            break;
        if (!dry && hackrf_is_streaming(dev) != HACKRF_TRUE && atomic_load(&reader_eof) &&
            atomic_load(&rpos) >= atomic_load(&wpos))
            break;
    }
    atomic_store(&stop_req, 1);
    if (!dry) {
        usleep(100000);
        hackrf_stop_tx(dev);
        if (stats) {
            uint64_t b = atomic_load(&sent_bytes);
            print_stats(now_s() - last, b - last_bytes, dev);
        }
        hackrf_close(dev);
        hackrf_exit();
    }
    pthread_join(rt, NULL);
    fprintf(stderr, "hackrf_tx_ram: sent %.3f GB of %.3f GB, host shortfalls %llu\n",
            atomic_load(&sent_bytes) / 1e9, file_size / 1e9, (unsigned long long)atomic_load(&host_short));
    if (noise_db > 0.0 || carrier_hz != 0.0)
        fprintf(stderr, "hackrf_tx_ram: processed (-N %.2f dB, -C %+.3f Hz), values clipped %llu\n", noise_db,
                carrier_hz, (unsigned long long)noise_clipped);
    return 0;
}
