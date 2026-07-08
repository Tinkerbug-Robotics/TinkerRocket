#pragma once

// Cross-core mutual exclusion for TR_FlightLog's shared index_/bitmap_ state
// (#388). The single flight-log instance is mutated from two pinned FreeRTOS
// tasks: the Core-0 flush task (prepareFlight / writeFrame / finalizeFlight)
// and the Core-1 oc_loop BLE handler (deleteFlight, plus the listFlights /
// readFlightPage readers). With no lock, a BLE delete concurrent with a
// flush-side prepare/finalize interleaves two erase+program sequences on the
// index snapshot blocks (1020/1021) and can tear BOTH copies — next boot the
// index CRC-fails both and all flight metadata is lost.
//
// TR_FlightLog is also compiled on the host for the gtest suite, which has no
// FreeRTOS, so the primitive is a FreeRTOS mutex on target and a std::mutex on
// host. The std::mutex is real (not a stub), so the host concurrency
// regression test exercises genuine mutual exclusion.

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#else
#include <mutex>
#endif

namespace tr_flightlog {

// Non-recursive mutex. TR_FlightLog never re-enters a guarded public method
// from another guarded public method — the two internal call chains
// (servicePendingPrepareFlight -> prepareFlight, writeFrame -> writePage) go
// through private *Locked helpers that assume the lock is already held — so a
// plain (cheaper) mutex is sufficient and matches the codebase idiom
// (TR_LogToFlash's spi_mutex_).
class FlightLogMutex {
public:
#ifdef ESP_PLATFORM
    // Safe to create during C++ static init: xSemaphoreCreateMutex only
    // allocates a queue structure and does not require the scheduler to be
    // running. A null handle (allocation failure) degrades lock()/unlock() to
    // no-ops — i.e. the pre-#388 unlocked behavior, never a crash.
    FlightLogMutex() : handle_(xSemaphoreCreateMutex()) {}
    ~FlightLogMutex() { if (handle_) vSemaphoreDelete(handle_); }
    void lock()   { if (handle_) xSemaphoreTake(handle_, portMAX_DELAY); }
    void unlock() { if (handle_) xSemaphoreGive(handle_); }
#else
    FlightLogMutex() = default;
    void lock()   { m_.lock(); }
    void unlock() { m_.unlock(); }
#endif

    FlightLogMutex(const FlightLogMutex&)            = delete;
    FlightLogMutex& operator=(const FlightLogMutex&) = delete;

private:
#ifdef ESP_PLATFORM
    SemaphoreHandle_t handle_;
#else
    std::mutex m_;
#endif
};

// RAII scoped lock. Held for the full logical flight-log operation so a
// multi-step index/bitmap mutation (e.g. FlightIndex::save's inspect -> erase
// -> program N pages) is atomic with respect to every other flight-log entry
// point on the other core.
class FlightLogLockGuard {
public:
    explicit FlightLogLockGuard(FlightLogMutex& m) : m_(m) { m_.lock(); }
    ~FlightLogLockGuard() { m_.unlock(); }

    FlightLogLockGuard(const FlightLogLockGuard&)            = delete;
    FlightLogLockGuard& operator=(const FlightLogLockGuard&) = delete;

private:
    FlightLogMutex& m_;
};

}  // namespace tr_flightlog
