// Host shim for ESP-IDF <driver/ledc.h>.
//
// Provides just enough of the LEDC (PWM) API for TR_ServoControl_ledc_mult to
// compile and run on a host toolchain (SIL / unit tests). There is no PWM
// peripheral on the host, so every ledc_* call is a no-op — but the servo
// controller's roll-command math (controlAngle cascade, rate cap, gain
// schedule, deg->us mapping) is pure and runs exactly as it does on the rocket.
//
// Enum *values* mirror the real esp-idf where the servo controller depends on
// them numerically: notably LEDC_TIMER_12_BIT == 12, because
// TR_ServoControl does `(1u << LEDC_RESOLUTION)` to size the duty range.
#ifndef HOST_SHIM_DRIVER_LEDC_H
#define HOST_SHIM_DRIVER_LEDC_H

#include <cstdint>

typedef enum {
    LEDC_LOW_SPEED_MODE = 0,
    LEDC_HIGH_SPEED_MODE,
    LEDC_SPEED_MODE_MAX,
} ledc_mode_t;

typedef enum {
    LEDC_CHANNEL_0 = 0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
    LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7,
    LEDC_CHANNEL_MAX,
} ledc_channel_t;

typedef enum {
    LEDC_TIMER_0 = 0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3,
    LEDC_TIMER_MAX,
} ledc_timer_t;

// Values MUST equal the bit count — TR_ServoControl uses (1u << LEDC_RESOLUTION).
typedef enum {
    LEDC_TIMER_1_BIT = 1, LEDC_TIMER_2_BIT, LEDC_TIMER_3_BIT, LEDC_TIMER_4_BIT,
    LEDC_TIMER_5_BIT, LEDC_TIMER_6_BIT, LEDC_TIMER_7_BIT, LEDC_TIMER_8_BIT,
    LEDC_TIMER_9_BIT, LEDC_TIMER_10_BIT, LEDC_TIMER_11_BIT, LEDC_TIMER_12_BIT,
    LEDC_TIMER_13_BIT, LEDC_TIMER_14_BIT,
} ledc_timer_bit_t;

typedef enum { LEDC_INTR_DISABLE = 0, LEDC_INTR_FADE_END } ledc_intr_type_t;
typedef enum { LEDC_AUTO_CLK = 0 } ledc_clk_cfg_t;

typedef struct {
    ledc_mode_t      speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t     timer_num;
    uint32_t         freq_hz;
    ledc_clk_cfg_t   clk_cfg;
} ledc_timer_config_t;

typedef struct {
    int              gpio_num;
    ledc_mode_t      speed_mode;
    ledc_channel_t   channel;
    ledc_intr_type_t intr_type;
    ledc_timer_t     timer_sel;
    uint32_t         duty;
    int              hpoint;
} ledc_channel_config_t;

// Minimal esp_err surface — TR_ServoControl::begin() checks the LEDC config
// results (esp_err_t / ESP_OK / esp_err_to_name).  Guarded: other shim headers
// (or a future esp_err.h shim) may define these too.
#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK 0
static inline const char* esp_err_to_name(esp_err_t) { return "ESP_OK"; }
#endif

// No-op stubs (host has no PWM peripheral). Return ESP_OK; the servo
// controller checks these in begin() and logs (no-op on host) on failure.
static inline esp_err_t ledc_timer_config(const ledc_timer_config_t*)        { return ESP_OK; }
static inline esp_err_t ledc_channel_config(const ledc_channel_config_t*)    { return ESP_OK; }
static inline esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t, uint32_t) { return ESP_OK; }
static inline esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t)        { return ESP_OK; }

#endif  // HOST_SHIM_DRIVER_LEDC_H
