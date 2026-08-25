/**
 * esp_heap_caps.h — Host stub for the ESP-IDF capability allocator.
 *
 * TR_LogToFlash::begin() asks for its ring with MALLOC_CAP_SPIRAM first
 * (#822) and falls back to MALLOC_CAP_INTERNAL.  On the host there is one
 * heap, so heap_caps_malloc ignores the caps and hands back malloc'd memory
 * — meaning the host always takes the FIRST branch when cfg.psram_ring_size
 * is set.  Tests that care which branch ran should leave psram_ring_size at
 * 0 (the default, and what every MRAM-fitted board passes) so the ring comes
 * from cfg.ring_buffer_size.
 */
#pragma once

#include <cstdint>
#include <cstdlib>

#define MALLOC_CAP_EXEC     (1 << 0)
#define MALLOC_CAP_32BIT    (1 << 1)
#define MALLOC_CAP_8BIT     (1 << 2)
#define MALLOC_CAP_DMA      (1 << 3)
#define MALLOC_CAP_SPIRAM   (1 << 10)
#define MALLOC_CAP_INTERNAL (1 << 11)
#define MALLOC_CAP_DEFAULT  (1 << 12)

inline void* heap_caps_malloc(size_t size, uint32_t caps)
{
    (void)caps;
    return malloc(size);
}

inline void heap_caps_free(void* p) { free(p); }

// Reported free space is a fixed plausible number: it only ever reaches an
// ESP_LOG argument, and a real query would make log lines nondeterministic.
inline size_t heap_caps_get_free_size(uint32_t caps)
{
    (void)caps;
    return 1u << 20;
}
