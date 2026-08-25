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

inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new _HostSemaphore(); }

inline void vSemaphoreDelete(SemaphoreHandle_t s) { delete s; }

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
