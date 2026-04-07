/**
 * compat.h — Lightweight Arduino-API shims backed by ESP-IDF
 *
 * Drop-in replacement for the subset of Arduino.h used across
 * TinkerRocket component libraries. Include this instead of
 * <Arduino.h> when building without the Arduino framework.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_attr.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Timing ─────────────────────────────────────────────── */

static inline uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline uint32_t micros(void)
{
    return (uint32_t)esp_timer_get_time();
}

static inline void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline void delayMicroseconds(uint32_t us)
{
    ets_delay_us(us);
}

static inline void yield(void)
{
    taskYIELD();
}

/* ── GPIO ───────────────────────────────────────────────── */

#ifndef OUTPUT
#define OUTPUT GPIO_MODE_OUTPUT
#endif
#ifndef INPUT
#define INPUT  GPIO_MODE_INPUT
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP GPIO_MODE_INPUT
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW  0
#endif

static inline void pinMode(uint8_t pin, uint8_t mode)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode         = (gpio_mode_t)mode;
    cfg.pull_up_en   = (mode == INPUT_PULLUP) ? GPIO_PULLUP_ENABLE
                                               : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);
}

static inline void digitalWrite(uint8_t pin, uint8_t val)
{
    gpio_set_level((gpio_num_t)pin, val);
}

static inline int digitalRead(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

/* ── Interrupts ─────────────────────────────────────────── */

#ifndef RISING
#define RISING  GPIO_INTR_POSEDGE
#endif
#ifndef FALLING
#define FALLING GPIO_INTR_NEGEDGE
#endif
#ifndef CHANGE
#define CHANGE  GPIO_INTR_ANYEDGE
#endif

/* Arduino attachInterrupt takes void(*)(void), but ESP-IDF gpio_isr_handler
   expects void(*)(void*).  We store the user's void(void) function pointer
   in the arg field and use a thin wrapper to call it. */
static void IRAM_ATTR _compat_isr_wrapper(void *arg)
{
    void (*fn)(void) = (void(*)(void))arg;
    fn();
}

static inline void attachInterrupt(uint8_t pin, void (*isr)(void), int mode)
{
    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)pin, _compat_isr_wrapper, (void*)isr);
    gpio_intr_enable((gpio_num_t)pin);
}

static inline void detachInterrupt(uint8_t pin)
{
    gpio_isr_handler_remove((gpio_num_t)pin);
    gpio_intr_disable((gpio_num_t)pin);
}

static inline void noInterrupts(void) { portDISABLE_INTERRUPTS(); }
static inline void interrupts(void)   { portENABLE_INTERRUPTS(); }

/* ── Arduino type aliases ───────────────────────────────── */

typedef bool     boolean;
typedef uint8_t  byte;

/* ── Misc Arduino macros ────────────────────────────────── */

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

/* NOTE: Arduino's min/max/abs macros are NOT defined here because they
   clash with std::min / std::max / std::abs in C++ headers.  Code that
   needs them should use std::min, std::max, or the C fabs/fabsf. */

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

#ifdef __cplusplus
}

/* ── SPI shim (C++ only) ───────────────────────────────── */

#include <driver/spi_master.h>

/* Arduino-style SPI mode / bit-order constants */
#ifndef SPI_MODE0
#define SPI_MODE0 0
#endif
#ifndef SPI_MODE1
#define SPI_MODE1 1
#endif
#ifndef SPI_MODE2
#define SPI_MODE2 2
#endif
#ifndef SPI_MODE3
#define SPI_MODE3 3
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef LSBFIRST
#define LSBFIRST 0
#endif

/**
 * Minimal SPISettings matching the Arduino API subset used by
 * TinkerRocket sensor drivers (ISM6HG256, BMP585, MMC5983MA, LogToFlash).
 */
struct SPISettings {
    uint32_t clock{1000000};
    uint8_t  bitOrder{MSBFIRST};
    uint8_t  dataMode{SPI_MODE0};

    SPISettings() = default;
    SPISettings(uint32_t clk, uint8_t order, uint8_t mode)
        : clock(clk), bitOrder(order), dataMode(mode) {}
};

/**
 * Lightweight SPIClass that wraps ESP-IDF spi_master for the Arduino
 * SPI API surface used across the codebase:
 *   begin / end / beginTransaction / endTransaction /
 *   transfer(byte) / transfer(buf,len) / transferBytes / writeBytes
 *
 * A single spi_device_handle is added per beginTransaction() settings
 * combination (cached for the most-recent settings to avoid repeated
 * spi_bus_add_device calls when the same sensor does back-to-back
 * transactions with identical settings).
 */
class SPIClass {
public:
    explicit SPIClass(spi_host_device_t host = SPI2_HOST) : host_(host) {}

    /* Initialise the SPI bus (equivalent to Arduino SPI.begin) */
    void begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t ss = -1)
    {
        (void)ss;
        spi_bus_config_t bus_cfg = {};
        bus_cfg.sclk_io_num     = sck;
        bus_cfg.miso_io_num     = miso;
        bus_cfg.mosi_io_num     = mosi;
        bus_cfg.quadwp_io_num   = -1;
        bus_cfg.quadhd_io_num   = -1;
        bus_cfg.max_transfer_sz = 4096;
        esp_err_t err = spi_bus_initialize(host_, &bus_cfg, SPI_DMA_CH_AUTO);
        if (err == ESP_OK) bus_inited_ = true;
    }

    void end()
    {
        for (int i = 0; i < dev_count_; i++)
            spi_bus_remove_device(dev_cache_[i].handle);
        dev_count_ = 0;
        dev_ = nullptr;
        if (bus_inited_) { spi_bus_free(host_); bus_inited_ = false; }
    }

    void beginTransaction(const SPISettings &s)
    {
        /* Re-use the existing device handle when settings match */
        if (dev_ && s.clock == cur_.clock &&
            s.bitOrder == cur_.bitOrder && s.dataMode == cur_.dataMode)
            return;
        cur_ = s;

        /* Check device cache (up to 4 devices with different settings) */
        for (int i = 0; i < dev_count_; i++) {
            if (dev_cache_[i].clock == s.clock &&
                dev_cache_[i].mode == s.dataMode &&
                dev_cache_[i].lsb == (s.bitOrder == LSBFIRST)) {
                dev_ = dev_cache_[i].handle;
                return;
            }
        }

        /* Create a new device for these settings */
        spi_device_interface_config_t dev_cfg = {};
        dev_cfg.clock_speed_hz = s.clock;
        dev_cfg.mode           = s.dataMode;
        dev_cfg.spics_io_num   = -1;          /* CS handled manually */
        dev_cfg.queue_size     = 1;
        dev_cfg.flags          = (s.bitOrder == LSBFIRST) ? SPI_DEVICE_BIT_LSBFIRST : 0;
        spi_device_handle_t h = nullptr;
        esp_err_t err = spi_bus_add_device(host_, &dev_cfg, &h);
        if (err != ESP_OK) {
            ESP_LOGE("SPI", "add_device failed: %s (clk=%lu mode=%u)",
                     esp_err_to_name(err), (unsigned long)s.clock, s.dataMode);
            dev_ = nullptr;
            return;
        }
        if (dev_count_ < MAX_CACHED_DEVS) {
            dev_cache_[dev_count_++] = {s.clock, s.dataMode,
                                        (uint8_t)(s.bitOrder == LSBFIRST), h};
        }
        dev_ = h;
    }

    void endTransaction() { /* device stays cached */ }

    /* Single-byte transfer (full-duplex) — uses inline tx_data/rx_data (no DMA needed) */
    uint8_t transfer(uint8_t data)
    {
        if (!dev_) { ESP_LOGE("SPI", "transfer: dev_ is NULL!"); return 0; }
        spi_transaction_t t = {};
        t.flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.length    = 8;
        t.tx_data[0] = data;
        esp_err_t err = spi_device_polling_transmit(dev_, &t);
        if (err != ESP_OK) {
            ESP_LOGE("SPI", "transfer err: %s mode=%u tx=0x%02X", esp_err_to_name(err), cur_.dataMode, data);
        }
        return t.rx_data[0];
    }

    /* In-place buffer transfer: reads into the same buffer (Arduino semantics) */
    void transfer(void *buf, uint32_t size)
    {
        if (!dev_ || !size) return;
        /* Both TX and RX buffers must be DMA-capable on ESP32-P4 */
        uint8_t *dma = (uint8_t *)heap_caps_malloc(size * 2, MALLOC_CAP_DMA);
        if (!dma) return;
        uint8_t *tx = dma;
        uint8_t *rx = dma + size;
        memcpy(tx, buf, size);
        spi_transaction_t t = {};
        t.length    = size * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        spi_device_polling_transmit(dev_, &t);
        memcpy(buf, rx, size);
        heap_caps_free(dma);
    }

    /* Separate TX/RX buffers */
    void transferBytes(const uint8_t *txBuf, uint8_t *rxBuf, uint32_t size)
    {
        spi_transaction_t t = {};
        t.length    = size * 8;
        t.tx_buffer = txBuf;
        t.rx_buffer = rxBuf;
        spi_device_polling_transmit(dev_, &t);
    }

    /* Write-only (no read) */
    void writeBytes(const uint8_t *data, uint32_t size)
    {
        spi_transaction_t t = {};
        t.length    = size * 8;
        t.tx_buffer = data;
        t.rx_buffer = nullptr;
        spi_device_polling_transmit(dev_, &t);
    }

private:
    static constexpr int MAX_CACHED_DEVS = 4;
    struct CachedDev {
        uint32_t clock;
        uint8_t  mode;
        uint8_t  lsb;
        spi_device_handle_t handle;
    };

    spi_host_device_t       host_;
    spi_device_handle_t     dev_       = nullptr;
    bool                    bus_inited_= false;
    SPISettings             cur_;
    CachedDev               dev_cache_[MAX_CACHED_DEVS] = {};
    int                     dev_count_ = 0;
};

/* Global SPI instance (matches Arduino's extern SPIClass SPI) */
inline SPIClass SPI(SPI2_HOST);

#endif /* __cplusplus */
