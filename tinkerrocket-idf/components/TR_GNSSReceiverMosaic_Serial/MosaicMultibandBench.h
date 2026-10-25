// MosaicMultibandBench.h — what each satellite's bands combined (MosaicMultiband.h) cost a CPU against the image's
// L1 adapter (MosaicRawEpoch.h), on recorded MeasEpoch blocks fed as the mosaic driver feeds them: a grid epoch
// (the 100 ms fused grid) built, each epoch between read for its Dopplers (the pairs), with the gyro's wind-up from
// the roll rate recorded beside each block. Each call is timed on whatever clock the caller hands it; the
// combiner's state lives in storage the caller lends (never a stack).
//
// The same code on the host and on the flight computer (TR_BENCH_MB_BLOCKS: at boot, in a task of its own), so
// the two can be set side by side. Header-only, bench only: nothing in flight calls it. The blocks themselves come
// from a file the bench build names, never from the repository (they hold a sky and so a place).
#ifndef MOSAIC_MULTIBAND_BENCH_H
#define MOSAIC_MULTIBAND_BENCH_H

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <new>

#include "MosaicMultiband.h"

namespace mosaic
{
namespace mbbench
{

using Clock = int64_t (*)();          // microseconds
using Print = void (*)(const char* line);

// One arm's times: a histogram in 10 us bins to 20 ms (the last bin open), the sum and the largest.
struct Times
{
    static constexpr int kBins = 2000;
    static constexpr double kBinUs = 10.0;
    uint32_t n = 0;
    double   sum_us = 0.0;
    double   max_us = 0.0;
    uint16_t hist[kBins] = {};
    void add(double us)
    {
        n++;
        sum_us += us;
        if (us > max_us) max_us = us;
        int b = static_cast<int>(us / kBinUs);
        if (b < 0) b = 0;
        if (b >= kBins) b = kBins - 1;
        if (hist[b] < 0xFFFF) hist[b]++;
    }
    double pct(double p) const
    {
        const uint32_t want = static_cast<uint32_t>(p * n);
        uint32_t c = 0;
        for (int b = 0; b < kBins; ++b)
        {
            c += hist[b];
            if (c > want) return (b + 0.5) * kBinUs;
        }
        return max_us;
    }
};

// The state both arms work in, lent by the caller (~40 kB: internal RAM where the driver's would be), and the
// arms' times (~16 kB, anywhere: touched only between calls).
struct Storage
{
    MosaicMultibandConfig cfg;
    MosaicMbScratch scr;
    MosaicMultibandStats st;
    MosaicMbOut out, out_prev;
    MosaicDopplerSet set_l1, set_mb;
    gnss_raw::RawEpoch epoch;
    uint8_t buf[4096];        // each block copied here first, as the driver's line buffer holds it
};
struct Arms
{
    Times l1_grid, l1_between, mb_grid, mb_between;
};

// Feed n blocks (each lens[i] bytes, the roll rate p_rps[i] about the nose when it came) through both arms in the
// same order, the L1 arm then the combined one on fresh state, and print what each cost. windup_k is the antenna's
// cycles a turn. Returns the combined arm's extra time over the L1 arm for one second of 20 Hz epochs (us).
inline double run(const char* name, const uint8_t* const* blocks, const uint16_t* lens, const float* p_rps, int n,
                  double windup_k, Storage& s, Arms& a, Clock clock, Print print, void (*yield)() = nullptr)
{
    char line[320];
    const float floors[GNSS_SYS_COUNT] = {25.0f, 25.0f, 25.0f, 25.0f};
    a.l1_grid = Times{}; a.l1_between = Times{}; a.mb_grid = Times{}; a.mb_between = Times{};
    int grid = 0, used = 0;
    for (int arm = 0; arm < 2; ++arm)
    {
        MosaicJumpState js;
        MosaicEpochInfo info;
        s.set_l1 = MosaicDopplerSet{};
        s.set_mb = MosaicDopplerSet{};
        if (arm == 1)
        {
            new (&s.scr) MosaicMbScratch();
            s.st = MosaicMultibandStats{};
            mosaicMbFlight(s.cfg, floors, 0.01, 0.075);
        }
        for (int i = 0; i < n; ++i)
        {
            if (lens[i] > sizeof(s.buf) || lens[i] < 20) continue;
            std::memcpy(s.buf, blocks[i], lens[i]);
            SbfMeasReader rd;
            if (rd.begin(s.buf, lens[i]) != SbfStatus::OK) continue;
            const bool on_grid = onFuseGrid(rd.header().tow_ms, 100);
            if (arm == 0)
            {
                used++;
                if (on_grid) grid++;
            }
            const MosaicMbWindupIn w = mbWindupFromGyro(true, p_rps[i], 0, windup_k);
            const int64_t t0 = clock();
            if (arm == 0)
            {
                if (on_grid)
                    (void)rawEpochFromMeasEpoch(s.buf, lens[i], s.epoch, js, info);
                else
                    (void)noteMeasEpochDopplers(s.buf, lens[i], js, info, s.set_l1);
            }
            else
            {
                if (on_grid)
                    (void)rawEpochFromMeasEpochMb(s.buf, lens[i], s.epoch, js, info, s.cfg, s.scr, s.st, s.out,
                                                  kMosaicNoArrival, w);
                else
                    (void)noteMeasEpochDopplersMb(s.buf, lens[i], js, info, s.set_mb, s.cfg, s.scr, s.st,
                                                  s.out_prev, kMosaicNoArrival, w);
            }
            const double us = static_cast<double>(clock() - t0);
            (arm == 0 ? (on_grid ? a.l1_grid : a.l1_between) : (on_grid ? a.mb_grid : a.mb_between)).add(us);
            if (on_grid) (arm == 0 ? s.set_l1 : s.set_mb).valid = false;   // one grid epoch's mate only, as the driver
            if (yield != nullptr && (i % 20) == 19) yield();
        }
    }
    auto row = [&](const char* what, const Times& t) {
        std::snprintf(line, sizeof(line), "%s %s: %u calls, mean %.0f us, median %.0f, p99 %.0f, max %.0f", name, what,
                      static_cast<unsigned>(t.n), t.n ? t.sum_us / t.n : 0.0, t.pct(0.5), t.pct(0.99), t.max_us);
        print(line);
    };
    row("L1 adapter, grid epochs", a.l1_grid);
    row("L1 adapter, epochs between", a.l1_between);
    row("bands combined, grid epochs", a.mb_grid);
    row("bands combined, epochs between", a.mb_between);
    const double secs = used * 0.05;
    const double l1 = a.l1_grid.sum_us + a.l1_between.sum_us, mb = a.mb_grid.sum_us + a.mb_between.sum_us;
    const double extra = secs > 0.0 ? (mb - l1) / secs : 0.0;
    std::snprintf(line, sizeof(line),
                  "%s: %d MeasEpochs (%.1f s at 20 Hz, %d on the grid) | a second of epochs: L1 %.0f us, combined "
                  "%.0f us (x%.1f), extra %.0f us = %.2f %% of a core | combined %u of %u anchors, voted out %u, "
                  "guard drops %u, anchor-only %u (few wind-up values %u)",
                  name, used, secs, grid, secs > 0.0 ? l1 / secs : 0.0, secs > 0.0 ? mb / secs : 0.0,
                  l1 > 0.0 ? mb / l1 : 0.0, extra, extra / 1e4, static_cast<unsigned>(s.st.sats_combined),
                  static_cast<unsigned>(s.st.sats), static_cast<unsigned>(s.st.voted),
                  static_cast<unsigned>(s.st.guard_rows),
                  static_cast<unsigned>(s.st.epochs_few_w + s.st.epochs_screen + s.st.epochs_xcheck + s.st.epochs_common),
                  static_cast<unsigned>(s.st.epochs_few_w));
    print(line);
    return extra;
}

// What the primitives cost this CPU, us an operation (2000 each): double pow, exp, sqrt and a multiply-add, and
// float sqrt -- a CPU without a double FPU (the P4, the S3) does the double ones in software.
inline void primitives(Clock clock, Print print)
{
    volatile double sink = 0.0;
    volatile float fsink = 0.0f;
    const int n = 2000;
    double x = 1.234567;
    int64_t t0 = clock();
    for (int i = 0; i < n; ++i) sink = sink + std::pow(10.0, -(x + i * 1e-4));
    const double t_pow = static_cast<double>(clock() - t0) / n;
    t0 = clock();
    for (int i = 0; i < n; ++i) sink = sink + std::exp(-(x + i * 1e-4));
    const double t_exp = static_cast<double>(clock() - t0) / n;
    t0 = clock();
    for (int i = 0; i < n; ++i) sink = sink + std::sqrt(x + i * 1e-4);
    const double t_sqrt = static_cast<double>(clock() - t0) / n;
    t0 = clock();
    double acc = 0.0;
    for (int i = 0; i < n; ++i) acc = acc * 0.999999 + (x + i * 1e-4);
    sink = acc;
    const double t_fma = static_cast<double>(clock() - t0) / n;
    float fx = 1.234567f;
    t0 = clock();
    for (int i = 0; i < n; ++i) fsink = fsink + std::sqrt(fx + i * 1e-4f);
    const double t_fsqrt = static_cast<double>(clock() - t0) / n;
    char line[200];
    std::snprintf(line, sizeof(line), "primitives, us an operation: double pow %.2f, exp %.2f, sqrt %.2f, "
                  "multiply-add %.3f; float sqrt %.3f", t_pow, t_exp, t_sqrt, t_fma, t_fsqrt);
    print(line);
    (void)sink;
    (void)fsink;
}

}  // namespace mbbench
}  // namespace mosaic

#endif
