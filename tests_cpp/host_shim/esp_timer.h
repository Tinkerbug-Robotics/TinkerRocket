/**
 * esp_timer.h — Host stub for the ESP-IDF high-resolution timer.
 *
 * TR_LogToFlash's stall instrumentation is built on esp_timer_get_time():
 * every LFS/NAND/sink call is wrapped in a start/end pair whose delta feeds
 * the *_max_us peaks and the #510 per-iteration ledger.  None of that is
 * under test on the host, but all of it has to compile and run, so this
 * returns a real monotonic microsecond clock rather than a constant — a
 * frozen clock would make every measured delta 0, which reads as "the
 * instrumentation works" whether or not it does.
 */
#pragma once

#include <chrono>
#include <cstdint>

inline int64_t esp_timer_get_time()
{
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return duration_cast<microseconds>(steady_clock::now() - t0).count();
}
