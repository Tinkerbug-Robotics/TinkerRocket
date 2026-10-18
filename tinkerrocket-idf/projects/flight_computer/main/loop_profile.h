#pragma once
// Bench section profiler for loop_fc (-DTR_LOOP_PROFILE=1). Never set for
// flight. The section list and the TR_LP_LAP macro are in TR_Compat's
// tr_loop_profile.h, so the EKF component can mark its own sub-steps; this is
// the implementation, included once from main.cpp.
//
// What it prints, every TR_LOOP_PROFILE_WINDOW_S seconds:
//   [LP] window ... | passes, pass mean/max | gated, ekf | IMU samples
//   [LP]   section  hits/s  us/pass  us/hit  max_us  core%  pm0%  pm1%
// us/pass is the section's mean cost per loop_fc pass, so the column adds up
// to the mean pass time, and core% to 100 (coverage). pm0/pm1 are the two
// Xtensa performance counters, as a share of the section's own cycles. The
// pair rotates every window through:
//   cfg 0: instruction-side stall cycles (all causes) | data-side stall cycles
//   cfg 1: instructions retired (pm0% = 100 x IPC)    | cycles at CINTLEVEL>0
//          (interrupt handlers and critical sections)
//   cfg 2: instruction "RAM/ROM busy" stall cycles   | WAITI cycles (idle task)
// The ESP32-S3's caches are outside the Xtensa core (XCHAL_ICACHE_SIZE 0), so a
// flash fetch through the cache stalls the core's instruction RAM port: cfg 2's
// pm0 is the cache-miss term, and cfg 0's pm0 adds the other fetch-side causes
// (FastL32R, iterative mul/div).
//
// Each window also prints the whole-pass counter shares ("[LP] total") and, on
// the S3, core 0's idle share ("[LP] core0 idle", its own counters read over
// IPC) -- the room there for work moved off core 1. The times are wall time on
// this core: interrupts, critical sections and any blocking (the 4 Hz I2C poll)
// land in the section they interrupt. tools/loop_profile_summary.py merges the
// windows of a capture into one table and compares captures side by side.
//
// At boot, before the loop, tr_lp_boot() prints the interrupt allocation table,
// the free heap, the PSRAM the heap got, and a microbenchmark: cycles per
// double and float math operation, one satellite's position and clock from its
// ephemeris (the raw-GNSS filter's per-satellite step, soft double on the P4),
// the queue and copy primitives the drain uses, and the instruction-cache miss
// penalty (a block of flash code run cold, after a larger block has evicted
// it, and warm). The profiler costs ~4 % of the loop.

#include "tr_loop_profile.h"

#if TR_LP_ENABLED

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_ipc.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "gnss/eph.h"     // TR_GNSSRaw: the raw-GNSS filter's satellite step, timed at boot
#include "gnss/types.h"
#if CONFIG_IDF_TARGET_ARCH_XTENSA
#include "xtensa-debug-module.h"
#define TR_LP_HAVE_PMU 1
#else
#define TR_LP_HAVE_PMU 0
#endif

#ifndef TR_LOOP_PROFILE_WINDOW_S
#define TR_LOOP_PROFILE_WINDOW_S 5
#endif

namespace tr_lp {

struct Acc {
    uint64_t cyc;
    uint64_t pm0;
    uint64_t pm1;
    uint32_t max_cyc;
    uint32_t hits;
};

static const char* const kNames[SECTION_COUNT] = {
#define TR_LP_NAME(id, name) name,
    TR_LP_SECTIONS(TR_LP_NAME)
#undef TR_LP_NAME
};

static DRAM_ATTR Acc acc[SECTION_COUNT];
static DRAM_ATTR uint32_t last_cyc, last_pm0, last_pm1;
static DRAM_ATTR uint32_t pass_start_cyc;
static DRAM_ATTR uint64_t pass_sum_cyc;
static DRAM_ATTR uint32_t pass_max_cyc, passes;
static DRAM_ATTR uint32_t samples;          // IMU samples drained (TR_LP_SAMPLE)
static DRAM_ATTR bool     started = false;
static int64_t  window_start_us = 0;
static uint32_t cpu_mhz = 240;
static uint8_t  pmu_cfg = 0;

struct PmuCfg {
    uint8_t  sel0;  uint16_t mask0;  uint8_t krn0;  uint8_t lvl0;
    uint8_t  sel1;  uint16_t mask1;  uint8_t krn1;  uint8_t lvl1;
    const char* label;
};
// select / mask values: xtensa/xt_perf_consts.h. krn=0, lvl=15 counts at every
// interrupt level; krn=1, lvl=0 counts only while CINTLEVEL > 0.
static const PmuCfg kPmuCfgs[] = {
    { 4, 0x01FF, 0, 15,   3, 0x01FE, 0, 15, "pm0=I-stall(all) pm1=D-stall(all)" },
    { 2, 0x8DFF, 0, 15,   0, 0x0001, 1,  0, "pm0=instructions(100xIPC) pm1=cycles@CINTLEVEL>0" },
    { 4, 0x0002, 0, 15,   6, 0x0100, 0, 15, "pm0=I-stall(RAM/ROM busy) pm1=WAITI(idle)" },
};
static constexpr uint8_t kPmuCfgCount = sizeof(kPmuCfgs) / sizeof(kPmuCfgs[0]);

#if TR_LP_HAVE_PMU
static inline __attribute__((always_inline)) uint32_t rer(uint32_t addr)
{
    uint32_t v;
    asm volatile("rer %0, %1" : "=r"(v) : "r"(addr));
    return v;
}
static inline __attribute__((always_inline)) void wer(uint32_t addr, uint32_t v)
{
    asm volatile("wer %0, %1" : : "r"(v), "r"(addr));
}
static uint32_t pmctrl(uint8_t sel, uint16_t mask, uint8_t krn, uint8_t lvl)
{
    return ((uint32_t)(lvl & PMCTRL_TRACELEVEL_MASK) << PMCTRL_TRACELEVEL_SHIFT) |
           ((uint32_t)(sel & PMCTRL_SELECT_MASK) << PMCTRL_SELECT_SHIFT) |
           ((uint32_t)(mask & PMCTRL_MASK_MASK) << PMCTRL_MASK_SHIFT) |
           ((uint32_t)(krn & 1) << PMCTRL_KRNLCNT_SHIFT);
}
// Must run on the core being profiled: each core has its own counters.
static void pmuConfigure(uint8_t cfg)
{
    const PmuCfg& c = kPmuCfgs[cfg];
    wer(ERI_PERFMON_PGM, 0);
    wer(ERI_PERFMON_PMCTRL0, pmctrl(c.sel0, c.mask0, c.krn0, c.lvl0));
    wer(ERI_PERFMON_PMCTRL0 + 4, pmctrl(c.sel1, c.mask1, c.krn1, c.lvl1));
    wer(ERI_PERFMON_PM0, 0);
    wer(ERI_PERFMON_PM0 + 4, 0);
    last_pm0 = 0;
    last_pm1 = 0;
    wer(ERI_PERFMON_PGM, PGM_PMEN);
}
#endif

#if TR_LP_HAVE_PMU
// Core 0's own counters (each core has a PMU), set and read from core 0 over
// IPC: PM0 = WAITI cycles (its idle task), PM1 = all cycles. Gives core 0's
// idle share per window -- the room there for work moved off core 1.
static uint32_t c0_last[2];
static bool c0_ok = false;
static void c0PmuInit(void*)
{
    wer(ERI_PERFMON_PGM, 0);
    wer(ERI_PERFMON_PMCTRL0, pmctrl(6, 0x0100, 0, 15));   // BUBBLES: WAITI
    wer(ERI_PERFMON_PMCTRL0 + 4, pmctrl(0, 0x0001, 0, 15)); // CYCLES
    wer(ERI_PERFMON_PM0, 0);
    wer(ERI_PERFMON_PM0 + 4, 0);
    wer(ERI_PERFMON_PGM, PGM_PMEN);
}
static void c0PmuRead(void* arg)
{
    uint32_t* out = static_cast<uint32_t*>(arg);
    out[0] = rer(ERI_PERFMON_PM0);
    out[1] = rer(ERI_PERFMON_PM0 + 4);
}
#endif

static void resetWindow()
{
    memset(acc, 0, sizeof(acc));
    pass_sum_cyc = 0;
    pass_max_cyc = 0;
    passes = 0;
    samples = 0;
    window_start_us = esp_timer_get_time();
}

}  // namespace tr_lp

#define TR_LP_SAMPLE() (++::tr_lp::samples)

// The lap itself: in IRAM, so it neither takes instruction-cache lines from the
// code it measures nor waits on a flash fetch of its own.
extern "C" void IRAM_ATTR tr_lp_lap(uint32_t s)
{
    using namespace tr_lp;
    const uint32_t c = esp_cpu_get_cycle_count();
#if TR_LP_HAVE_PMU
    const uint32_t p0 = rer(ERI_PERFMON_PM0);
    const uint32_t p1 = rer(ERI_PERFMON_PM0 + 4);
#else
    const uint32_t p0 = 0, p1 = 0;
#endif
    if (!started) return;
    Acc& a = acc[s];
    const uint32_t d = c - last_cyc;
    a.cyc += d;
    if (d > a.max_cyc) a.max_cyc = d;
    a.hits++;
    a.pm0 += (uint32_t)(p0 - last_pm0);
    a.pm1 += (uint32_t)(p1 - last_pm1);
    last_cyc = c;
    last_pm0 = p0;
    last_pm1 = p1;
    if (s == YIELD)   // first lap of a pass
    {
        const uint32_t pc = c - pass_start_cyc;
        pass_start_cyc = c;
        pass_sum_cyc += pc;
        if (pc > pass_max_cyc) pass_max_cyc = pc;
        passes++;
    }
}

namespace tr_lp {

static void startCounting()
{
#if TR_LP_HAVE_PMU
    pmuConfigure(pmu_cfg);
    c0_ok = (esp_ipc_call_blocking(0, c0PmuInit, nullptr) == ESP_OK) &&
            (esp_ipc_call_blocking(0, c0PmuRead, c0_last) == ESP_OK);
#endif
    resetWindow();
    last_cyc = esp_cpu_get_cycle_count();
    pass_start_cyc = last_cyc;
    started = true;
}

// x10 fixed point, for printing without float formatting.
static inline unsigned x10(uint64_t num, uint64_t den)
{
    return den ? (unsigned)((num * 10u + den / 2u) / den) : 0u;
}

static void report()
{
    const int64_t now_us = esp_timer_get_time();
    const uint64_t win_us = (uint64_t)(now_us - window_start_us);
    if (win_us < (uint64_t)TR_LOOP_PROFILE_WINDOW_S * 1000000ull) return;

    uint64_t total = 0;
    for (int i = 0; i < SECTION_COUNT; i++) total += acc[i].cyc;
    const uint64_t win_cyc = win_us * cpu_mhz;
    printf("[LP] window %u.%01u s cfg %u (%s) | passes %u = %u.%01u/s, mean %u.%01u us, max %u us"
           " | gated %u ekf %u | imu %u/s %u.%01u/pass | coverage %u.%01u %%\n",
           (unsigned)(win_us / 1000000u), (unsigned)((win_us / 100000u) % 10u),
           (unsigned)pmu_cfg, kPmuCfgs[pmu_cfg].label,
           (unsigned)passes,
           x10((uint64_t)passes * 1000000u, win_us) / 10u, x10((uint64_t)passes * 1000000u, win_us) % 10u,
           x10(pass_sum_cyc, (uint64_t)passes * cpu_mhz) / 10u, x10(pass_sum_cyc, (uint64_t)passes * cpu_mhz) % 10u,
           (unsigned)(pass_max_cyc / cpu_mhz),
           (unsigned)acc[IMU_MEAN].hits, (unsigned)acc[EKF_NAV].hits,
           (unsigned)((uint64_t)samples * 1000000u / (win_us ? win_us : 1)),
           x10(samples, passes) / 10u, x10(samples, passes) % 10u,
           x10(total * 100u, win_cyc) / 10u, x10(total * 100u, win_cyc) % 10u);
    printf("[LP]   %-15s %8s %9s %9s %8s %6s %6s %6s\n",
           "section", "hits/s", "us/pass", "us/hit", "max_us", "core%", "pm0%", "pm1%");
    for (int i = 0; i < SECTION_COUNT; i++)
    {
        const Acc& a = acc[i];
        if (a.hits == 0) continue;
        const unsigned hz   = x10((uint64_t)a.hits * 1000000u, win_us);
        const unsigned upp  = x10(a.cyc, (uint64_t)(passes ? passes : 1) * cpu_mhz);
        const unsigned uph  = x10(a.cyc, (uint64_t)a.hits * cpu_mhz);
        const unsigned core = x10(a.cyc * 100u, total);
        const unsigned p0   = x10(a.pm0 * 100u, a.cyc);
        const unsigned p1   = x10(a.pm1 * 100u, a.cyc);
        printf("[LP]   %-15s %6u.%01u %7u.%01u %7u.%01u %8u %4u.%01u %4u.%01u %4u.%01u\n",
               kNames[i], hz / 10u, hz % 10u, upp / 10u, upp % 10u, uph / 10u, uph % 10u,
               (unsigned)(a.max_cyc / cpu_mhz), core / 10u, core % 10u,
               p0 / 10u, p0 % 10u, p1 / 10u, p1 % 10u);
    }
    // Whole-window counter shares, the line a script compares across configs.
    uint64_t pm0 = 0, pm1 = 0;
    for (int i = 0; i < SECTION_COUNT; i++) { pm0 += acc[i].pm0; pm1 += acc[i].pm1; }
    printf("[LP] total cfg %u pm0 %u.%01u %% pm1 %u.%01u %% of %u Mcycles\n",
           (unsigned)pmu_cfg, x10(pm0 * 100u, total) / 10u, x10(pm0 * 100u, total) % 10u,
           x10(pm1 * 100u, total) / 10u, x10(pm1 * 100u, total) % 10u,
           (unsigned)(total / 1000000u));

#if TR_LP_HAVE_PMU
    if (c0_ok)
    {
        uint32_t c0_now[2] = {0, 0};
        if (esp_ipc_call_blocking(0, c0PmuRead, c0_now) == ESP_OK)
        {
            const uint32_t dw = c0_now[0] - c0_last[0];
            const uint32_t dc = c0_now[1] - c0_last[1];
            printf("[LP] core0 idle %u.%01u %% (WAITI %u of %u Mcycles)\n",
                   x10((uint64_t)dw * 100u, dc) / 10u, x10((uint64_t)dw * 100u, dc) % 10u,
                   (unsigned)(dw / 1000000u), (unsigned)(dc / 1000000u));
            c0_last[0] = c0_now[0];
            c0_last[1] = c0_now[1];
        }
    }
#endif
    pmu_cfg = (uint8_t)((pmu_cfg + 1u) % kPmuCfgCount);
#if TR_LP_HAVE_PMU
    pmuConfigure(pmu_cfg);
#endif
    resetWindow();
}

// ── Boot microbenchmark ──────────────────────────────────────────────────
static volatile double   vd_in = 0.7123, vd_in2 = 1.3, vd_out;
static volatile float    vf_in = 0.7123f, vf_in2 = 1.3f, vf_out;
static volatile uint32_t vu_out;

#define TR_LP_BENCH(label, n, stmt)                                         \
    do {                                                                    \
        for (int _w = 0; _w < 2; _w++) {   /* pass 0 warms the cache */    \
            const uint32_t _t0 = esp_cpu_get_cycle_count();                 \
            for (int _i = 0; _i < (n); _i++) { stmt; }                      \
            const uint32_t _dt = esp_cpu_get_cycle_count() - _t0;           \
            if (_w == 1) printf("[LP-BENCH] %-28s %6u cycles/op\n", label,  \
                                (unsigned)(_dt / (n)));                     \
        }                                                                   \
    } while (0)

#if CONFIG_IDF_TARGET_ESP32S3
// Straight-line flash code for the miss-penalty test. Whether nop assembles to
// 2 or 3 bytes is the assembler's choice, so read the two sizes from the ELF
// (nm -S) rather than from these counts: ~4 KB and ~48 KB at 2 bytes.
static void __attribute__((noinline)) benchColdBlock(void)
{
    asm volatile(".rept 2048\n\tnop\n\t.endr");
}
static void __attribute__((noinline)) benchEvictBlock(void)
{
    asm volatile(".rept 24576\n\tnop\n\t.endr");
}
static uint32_t IRAM_ATTR timeCall(void (*fn)(void))
{
    const uint32_t t0 = esp_cpu_get_cycle_count();
    fn();
    return esp_cpu_get_cycle_count() - t0;
}
#endif

// One satellite as RawMeasBuilder solves it (RawMeasBuilder.cpp satState): the
// clock at the transmit time, then the position at t and t +- 0.5 s for the
// velocity and acceleration. All soft double on the P4, whose FPU is single
// precision, so this is the filter's per-satellite cost the P4 has to price.
static volatile double vd_t_sv = 302430.0;
static double __attribute__((noinline)) benchSatSolve(const gps_eph_t* e)
{
    const double t_sv = vd_t_sv;
    const double t_tx = t_sv - gps_sat_clock(e, t_sv);
    double rs[3], rs1[3], rs2[3], dts, dts1, dts2;
    gps_sat_pos(e, t_tx, rs, nullptr, &dts);
    gps_sat_pos(e, t_tx + 0.5, rs2, nullptr, &dts2);
    gps_sat_pos(e, t_tx - 0.5, rs1, nullptr, &dts1);
    return rs[0] + rs1[1] + rs2[2] + dts + dts1 + dts2;
}

static void boot()
{
    cpu_mhz = (uint32_t)(esp_clk_cpu_freq() / 1000000);
    printf("[LP] flight-loop profiler on core %d, CPU %u MHz, window %d s\n",
           (int)xPortGetCoreID(), (unsigned)cpu_mhz, (int)TR_LOOP_PROFILE_WINDOW_S);
    esp_intr_dump(stdout);
    // Room for moving hot code into internal RAM (IRAM_ATTR / noflash), after setup.
    printf("[LP] heap: internal free %u largest %u | psram free %u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    // Whether the build asked for PSRAM and what the heap got: with
    // CONFIG_SPIRAM_IGNORE_NOTFOUND a part that did not come up leaves 0 here.
    printf("[LP] psram: CONFIG_SPIRAM %s | in the heap %u KB, largest block %u B\n",
#if CONFIG_SPIRAM
           "on",
#else
           "off",
#endif
           (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024u),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

    // Loop overhead of a volatile read + write, to subtract by eye.
    TR_LP_BENCH("baseline (volatile r/w)", 2000, vd_out = vd_in);
    TR_LP_BENCH("double add", 2000, vd_out = vd_in + vd_in2);
    TR_LP_BENCH("double mul", 2000, vd_out = vd_in * vd_in2);
    TR_LP_BENCH("double mul-add", 2000, vd_out = vd_in * vd_in2 + vd_in);
    TR_LP_BENCH("double div", 2000, vd_out = vd_in / vd_in2);
    TR_LP_BENCH("double sqrt", 1000, vd_out = sqrt(vd_in));
    TR_LP_BENCH("double sin", 1000, vd_out = sin(vd_in));
    TR_LP_BENCH("double cos", 1000, vd_out = cos(vd_in));
    TR_LP_BENCH("double atan2", 1000, vd_out = atan2(vd_in, vd_in2));
    TR_LP_BENCH("double->float", 2000, vf_out = (float)vd_in);
    TR_LP_BENCH("float->double", 2000, vd_out = (double)vf_in);
    TR_LP_BENCH("float mul-add", 2000, vf_out = vf_in * vf_in2 + vf_in);
    TR_LP_BENCH("float div", 2000, vf_out = vf_in / vf_in2);
    TR_LP_BENCH("sqrtf", 2000, vf_out = sqrtf(vf_in));
    TR_LP_BENCH("sinf", 1000, vf_out = sinf(vf_in));
    TR_LP_BENCH("cosf", 1000, vf_out = cosf(vf_in));
    TR_LP_BENCH("atan2f", 1000, vf_out = atan2f(vf_in, vf_in2));
    TR_LP_BENCH("asinf", 1000, vf_out = asinf(vf_in * 0.5f));
    TR_LP_BENCH("powf", 1000, vf_out = powf(vf_in, 0.190295f));
    TR_LP_BENCH("expf", 1000, vf_out = expf(vf_in));
    TR_LP_BENCH("esp_timer_get_time", 2000, vu_out = (uint32_t)esp_timer_get_time());

    {
        // A GPS-like orbit (made-up elements, no real satellite's): eccentric
        // enough that the Kepler loop runs its usual handful of iterations.
        static gps_eph_t eph;
        eph.valid = 1;
        eph.sys = GNSS_SYS_GPS;
        eph.prn = 1;
        eph.toe = eph.toc = 302400.0;
        eph.sqrt_a = 5153.6;
        eph.e = 0.012;
        eph.i0 = 0.96;
        eph.omega0 = 1.1;
        eph.omega = 0.7;
        eph.m0 = 2.3;
        eph.delta_n = 4.5e-9;
        eph.idot = 1.0e-10;
        eph.omega_dot = -8.0e-9;
        eph.cuc = 1.0e-6;
        eph.cus = 8.0e-6;
        eph.crc = 200.0;
        eph.crs = 20.0;
        eph.cic = 1.0e-7;
        eph.cis = -1.0e-7;
        eph.af0 = 1.0e-4;
        eph.af1 = 1.0e-12;
        double pos[3], clk;
        TR_LP_BENCH("eph sat pos+clock", 200,
                    gps_sat_pos(&eph, vd_t_sv, pos, nullptr, &clk); vd_out = pos[0] + clk);
        TR_LP_BENCH("eph sat solve (builder's)", 50, vd_out = benchSatSolve(&eph));
    }

    {
        static uint8_t src[226], dst[226];
        TR_LP_BENCH("memcpy 226 B", 2000, memcpy(dst, src, sizeof(dst)); vu_out = dst[7]);
        TR_LP_BENCH("memset 226 B", 2000, memset(dst, 0, sizeof(dst)); vu_out = dst[7]);
        QueueHandle_t q = xQueueCreate(4, 226);
        QueueHandle_t q32 = xQueueCreate(4, 32);
        if (q && q32)
        {
            TR_LP_BENCH("xQueueSend+Receive 226 B", 1000,
                        (void)xQueueSend(q, src, 0); (void)xQueueReceive(q, dst, 0));
            TR_LP_BENCH("xQueueSend+Receive 32 B", 1000,
                        (void)xQueueSend(q32, src, 0); (void)xQueueReceive(q32, dst, 0));
            TR_LP_BENCH("xQueueReceive empty", 2000, (void)xQueueReceive(q, dst, 0));
        }
        if (q) vQueueDelete(q);
        if (q32) vQueueDelete(q32);
    }

#if CONFIG_IDF_TARGET_ESP32S3
    // Instruction-cache miss penalty: run the small block cold (after the large
    // one has pushed it out of the 16/32 KB cache) and warm, 8 times.
    {
        uint32_t cold_min = UINT32_MAX, warm_min = UINT32_MAX, evict_min = UINT32_MAX;
        for (int k = 0; k < 8; k++)
        {
            const uint32_t e = timeCall(benchEvictBlock);
            const uint32_t c = timeCall(benchColdBlock);
            const uint32_t w = timeCall(benchColdBlock);
            if (e < evict_min) evict_min = e;
            if (c < cold_min) cold_min = c;
            if (w < warm_min) warm_min = w;
        }
        printf("[LP-BENCH] icache: small block cold %u warm %u cycles; large block cold %u cycles"
               " (cache %u KB, line %u B, flash %s)\n",
               (unsigned)cold_min, (unsigned)warm_min, (unsigned)evict_min,
               (unsigned)(CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE / 1024),
               (unsigned)CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE,
#if CONFIG_ESPTOOLPY_FLASHMODE_QIO
               "qio"
#elif CONFIG_ESPTOOLPY_FLASHMODE_DIO
               "dio"
#else
               "other"
#endif
        );
    }
#endif
    startCounting();
}

}  // namespace tr_lp

// Called by the flight task before its first loop_fc() pass.
static inline void tr_lp_boot() { tr_lp::boot(); }
// Called at the end of every loop_fc() pass; prints at most once a window.
static inline void tr_lp_maybe_report()
{
    tr_lp::report();
    TR_LP_LAP(REPORT);
}

#else  // !TR_LP_ENABLED

#define TR_LP_SAMPLE() ((void)0)
static inline void tr_lp_boot() {}
static inline void tr_lp_maybe_report() {}

#endif
