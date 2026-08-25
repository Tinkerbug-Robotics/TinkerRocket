/**
 * esp_system.h — Host stub for the handful of esp_system entry points the
 * component libraries touch.  Like heap_caps_get_free_size, the free-heap
 * figure only ever reaches a log argument.
 */
#pragma once

#include <cstdint>

inline uint32_t esp_get_free_heap_size() { return 1u << 20; }
inline uint32_t esp_get_minimum_free_heap_size() { return 1u << 20; }
