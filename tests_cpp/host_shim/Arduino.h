/**
 * Arduino.h — Host-side shim for building TinkerRocket C++ libraries
 *             on desktop (macOS/Linux) without any ESP-IDF or Arduino deps.
 *
 * Provides the minimal API surface used by:
 *   TR_Coordinates, TR_KinematicChecks, TR_Sensor_Data_Converter,
 *   CRC library (CrcDefines.h)
 *
 * Pattern follows tinkerrocket-idf/components/TR_Compat/include/compat.h
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <algorithm>

// --------------- Timing (injectable for testing) ---------------

namespace _host_shim {
    inline uint32_t& mock_millis_ref() {
        static uint32_t val = 0;
        return val;
    }
    inline uint32_t& mock_micros_ref() {
        static uint32_t val = 0;
        return val;
    }
}

inline void setMockMillis(uint32_t ms) { _host_shim::mock_millis_ref() = ms; }
inline void setMockMicros(uint32_t us) { _host_shim::mock_micros_ref() = us; }

inline uint32_t millis() { return _host_shim::mock_millis_ref(); }
inline uint32_t micros() { return _host_shim::mock_micros_ref(); }

inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void yield() {}

// --------------- Math constants ---------------

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef TWO_PI
#define TWO_PI (2.0 * PI)
#endif

#ifndef HALF_PI
#define HALF_PI (PI / 2.0)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / PI)
#endif

// --------------- Arduino macros ---------------

#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef abs
#define abs(x) ((x) >= 0 ? (x) : -(x))
#endif

#ifndef map
#define map(x, in_min, in_max, out_min, out_max) \
    (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
#endif

#ifndef radians
#define radians(deg) ((deg) * PI / 180.0)
#endif

#ifndef degrees
#define degrees(rad) ((rad) * 180.0 / PI)
#endif

// --------------- Type aliases ---------------

typedef bool    boolean;
typedef uint8_t byte;

// --------------- GPIO stubs ---------------

#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT
#define INPUT  0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW  0
#endif

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int  digitalRead(uint8_t) { return 0; }

// --------------- Serial stub ---------------

struct SerialStub {
    void begin(unsigned long) {}
    void print(const char*) {}
    void print(int) {}
    void print(float) {}
    void print(double) {}
    void println(const char* s = "") { (void)s; }
    void println(int) {}
    void println(float) {}
    void println(double) {}
    void printf(const char*, ...) {}
};

inline SerialStub Serial;

// --------------- F() macro (Arduino flash string) ---------------

#ifndef F
#define F(x) (x)
#endif
