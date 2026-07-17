#ifndef FLUSH_ITER_LEDGER_H
#define FLUSH_ITER_LEDGER_H

#include <stdint.h>

// #510: per-iteration accounting for flushTaskLoop.
//
// The 2026-07-14 bench flight logged a 588 ms flush-task "STALL" at logging
// activation of which only ~11 ms was visible to the existing instrumentation
// — because every accumulator (write_max_us_ etc.) is a per-op MAXIMUM. The
// activation iteration drains the whole ~97 KB prelaunch ring in one pass:
// ~24 pages of MRAM ringPop + NAND program, each individually well under the
// 100 ms per-op stall threshold, so hundreds of ms of legitimate work simply
// vanished from the breakdown. In exactly that window the ring has only ~0.5 s
// of overflow headroom, so a *genuine* blocker there must be attributable.
//
// This ledger sums wall time per iteration SECTION (staging / pre-create open
// / hook / activate / drain / end-of-flight), plus sub-details accumulated
// inside flushRingToNand (pages, bytes, MRAM-pop time, write time). classify()
// then splits long iterations into:
//   LongAccounted   — the section timers explain the time: designed work such
//                     as the arm-time 80-block erase (hook) or the activation
//                     ring drain. Logged INFO, not WARN (#510 item 3).
//   LongUnaccounted — a real blind spot remains. Logged WARN with the
//                     breakdown, naming where the time did NOT go.
//
// Pure arithmetic (no clock, no FreeRTOS) — host-tested in
// test_flush_iter_ledger.cpp, same pattern as MramDirtyPolicy (#417).
struct FlushIterLedger
{
    // Section wall-times (µs), one bracket per flushTaskLoop section.
    uint32_t staging_us   = 0;  // flushStagingIfStale
    uint32_t open_us      = 0;  // PRELAUNCH pre-create openLogSession
    uint32_t hook_us      = 0;  // flush_task_hook (arm-time prepareFlight: auto-evict + 80-block erase)
    uint32_t activate_us  = 0;  // start_logging_requested block (activateLogging)
    uint32_t drain_us     = 0;  // normal-flush flushRingToNand call
    uint32_t endflight_us = 0;  // end_flight_requested drain + rename + close

    // Sub-details accumulated inside flushRingToNand, under whichever section
    // invoked it (drain_us normally, endflight_us during the final drain).
    // They explain section time and are NOT added to accountedUs().
    uint32_t drain_pages = 0;   // pages shipped to NAND
    uint32_t drain_bytes = 0;   // ring bytes drained into those pages
    uint32_t pop_us      = 0;   // MRAM ringPeekAt/ringPop wall time
    uint32_t write_us    = 0;   // write_sink / lfs_file_write wall time

    void reset() { *this = FlushIterLedger{}; }

    // Sum of the section timers — everything the iteration is KNOWN to have
    // spent. iter_wall - accountedUs() is the remaining blind spot.
    uint32_t accountedUs() const
    {
        return staging_us + open_us + hook_us + activate_us + drain_us
             + endflight_us;
    }

    // Clamped at 0: section brackets sit inside the iteration bracket, but
    // the extra esp_timer reads can make the sum land a hair over.
    uint32_t unaccountedUs(uint32_t iter_us) const
    {
        const uint32_t acc = accountedUs();
        return (iter_us > acc) ? (iter_us - acc) : 0;
    }

    // Drain-section time not explained by MRAM pops or writes — isolates the
    // FL diagnostic prints / loop overhead (the console-backpressure suspect).
    // Meaningful when drain_us dominates; clamped at 0 otherwise (e.g. an
    // end-of-flight iteration books its pops under endflight_us instead).
    uint32_t drainOtherUs() const
    {
        const uint32_t sub = pop_us + write_us;
        return (drain_us > sub) ? (drain_us - sub) : 0;
    }

    enum class Verdict : uint8_t
    {
        Quiet,            // iteration at/under the stall threshold — no print
        LongAccounted,    // long, but the section timers explain it (INFO)
        LongUnaccounted,  // long with a real blind spot remaining (WARN)
    };

    // Thresholding matches the pre-#510 stall check: a print fires only when
    // iter_us exceeds threshold_us, and it stays a WARN only when the
    // unaccounted share alone would also exceed it.
    static Verdict classify(uint32_t iter_us, uint32_t accounted_us,
                            uint32_t threshold_us)
    {
        if (iter_us <= threshold_us) return Verdict::Quiet;
        const uint32_t unacc = (iter_us > accounted_us) ? (iter_us - accounted_us) : 0;
        return (unacc <= threshold_us) ? Verdict::LongAccounted
                                       : Verdict::LongUnaccounted;
    }
};

#endif  // FLUSH_ITER_LEDGER_H
