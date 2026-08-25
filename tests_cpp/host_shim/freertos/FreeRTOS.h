/**
 * freertos/FreeRTOS.h — Host stub for the FreeRTOS core types and the
 *                       critical-section spinlock.
 *
 * Host tests run the component SINGLE-THREADED (see freertos/task.h for how
 * the flush "task" is driven), so portENTER_CRITICAL / portEXIT_CRITICAL are
 * no-ops and the mutexes in semphr.h never block.  That is a deliberate
 * limitation, not an oversight: TR_LogToFlash's cross-core hazards (#74's
 * rb_head clobber, #365's consume-on-observe request flags, #370's
 * push/pop serialization) are RACES, and a single-threaded harness cannot
 * observe a race.  What it can observe is the accounting those paths
 * maintain, which is what these tests pin.
 */
#pragma once

#include <cstdint>

typedef int      BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define pdFAIL  0

#define portMAX_DELAY ((TickType_t)0xFFFFFFFFu)

#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ 1000
#endif

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

// The real portMUX_TYPE is a spinlock guarding a cross-core critical section.
// With one thread there is nothing to spin against, so it degrades to an
// empty struct the compiler can optimize away entirely.
typedef struct { int _unused; } portMUX_TYPE;

#define portMUX_INITIALIZER_UNLOCKED { 0 }

#define portENTER_CRITICAL(mux) do { (void)(mux); } while (0)
#define portEXIT_CRITICAL(mux)  do { (void)(mux); } while (0)
#define portENTER_CRITICAL_ISR(mux) portENTER_CRITICAL(mux)
#define portEXIT_CRITICAL_ISR(mux)  portEXIT_CRITICAL(mux)

#define taskYIELD() do {} while (0)
