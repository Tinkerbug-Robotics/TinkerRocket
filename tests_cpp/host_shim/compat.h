/**
 * compat.h — Host-side forwarder for TinkerRocket component headers
 *            that include <compat.h> when building against the real
 *            ESP-IDF TR_Compat component.
 *
 * On the host the real compat.h cannot be used (it pulls in
 * esp_timer.h, FreeRTOS, driver/gpio.h, etc. — all ESP-only).
 * We forward to the existing host_shim/Arduino.h, which provides the
 * subset of the Arduino API our tests exercise — BUT we then #undef
 * the preprocessor macros that conflict with std:: names (`max`,
 * `min`, `abs`, `constrain`). Post-migration component code uses
 * `std::max` etc. instead of the Arduino macros.
 */
#pragma once

#include "Arduino.h"

// Mirror the real TR_Compat compat.h on the device, which defines
// `constrain` as a macro but does NOT poison `min`/`max`/`abs`.
// Post-migration component code (e.g. TR_KinematicChecks) uses
// std::max; leaving those as macros breaks those calls at parse time.
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#ifdef abs
#undef abs
#endif
// NB: `constrain` intentionally kept — TR_PID and others rely on it.
