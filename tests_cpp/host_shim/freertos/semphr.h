/**
 * freertos/semphr.h — Host stub for the FreeRTOS mutex API.
 *
 * Single-threaded harness (see freertos/FreeRTOS.h), so a take never blocks.
 * The handle is a live counter rather than a dummy pointer so an unbalanced
 * take/give shows up as a wrong count instead of passing silently — the
 * component's spiAcquire/spiRelease and push_mutex_ pairs are supposed to
 * balance, and a leaked hold would deadlock on target.
 */
#pragma once

#include "FreeRTOS.h"

struct _HostSemaphore {
    int held = 0;
};

typedef _HostSemaphore* SemaphoreHandle_t;

namespace _host_shim {
/// Mutexes created and not yet deleted.  A component that creates its
/// mutexes in begin() must not create them AGAIN when begin() is re-entered
/// after a failure (#1228) — on target that is a heap leak per attempt, and
/// here it is a count that moved.
inline int& liveMutexes()
{
    static int n = 0;
    return n;
}
}  // namespace _host_shim

inline SemaphoreHandle_t xSemaphoreCreateMutex()
{
    _host_shim::liveMutexes()++;
    return new _HostSemaphore();
}

inline void vSemaphoreDelete(SemaphoreHandle_t s)
{
    if (s) _host_shim::liveMutexes()--;
    delete s;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t)
{
    if (!s) return pdFALSE;
    s->held++;
    return pdTRUE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s)
{
    if (!s) return pdFALSE;
    s->held--;
    return pdTRUE;
}
