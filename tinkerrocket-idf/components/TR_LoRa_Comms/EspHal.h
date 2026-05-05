/**
 * EspHal.h — RadioLib hardware abstraction layer for ESP32-S3
 *
 * Uses ESP-IDF native drivers (spi_master, gpio, esp_timer) instead of
 * the Arduino HAL.  Designed for the LLCC68 radio on TinkerRocket.
 *
 * Based on the RadioLib ESP-IDF example HAL, rewritten to use the
 * portable spi_master.h API so it works on ESP32-S3 (and any other
 * ESP32 variant) without touching SoC registers directly.
 */
#pragma once

#include <RadioLib.h>

#include <cstring>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

// Value mappings passed to RadioLibHal base class
#define ESPHAL_INPUT    GPIO_MODE_INPUT
#define ESPHAL_OUTPUT   GPIO_MODE_OUTPUT
#define ESPHAL_LOW      0
#define ESPHAL_HIGH     1
#define ESPHAL_RISING   GPIO_INTR_POSEDGE
#define ESPHAL_FALLING  GPIO_INTR_NEGEDGE

class EspHal : public RadioLibHal {
public:
    /**
     * @param sck   SPI clock pin
     * @param miso  SPI MISO pin
     * @param mosi  SPI MOSI pin
     * @param spiHost  SPI host to use (SPI2_HOST or SPI3_HOST)
     * @param spiFreqHz  SPI clock frequency in Hz (default 2 MHz)
     */
    EspHal(int8_t sck, int8_t miso, int8_t mosi,
           spi_host_device_t spiHost = SPI2_HOST,
           uint32_t spiFreqHz = 2000000)
        : RadioLibHal(ESPHAL_INPUT, ESPHAL_OUTPUT, ESPHAL_LOW, ESPHAL_HIGH,
                      ESPHAL_RISING, ESPHAL_FALLING),
          spiSCK(sck), spiMISO(miso), spiMOSI(mosi),
          spiHostDev(spiHost), spiClockHz(spiFreqHz) {}

    // ── Lifecycle ───────────────────────────────────────────

    void init() override {
        spiBegin();
    }

    void term() override {
        spiEnd();
    }

    // ── GPIO ────────────────────────────────────────────────

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;

        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode = (gpio_mode_t)mode;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin == RADIOLIB_NC) return;
        gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin == RADIOLIB_NC) return 0;
        return gpio_get_level((gpio_num_t)pin);
    }

    // ── Interrupts ──────────────────────────────────────────

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void),
                         uint32_t mode) override {
        if (interruptNum == RADIOLIB_NC) return;

        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

        // Configure the pin for interrupt
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)mode);

        // RadioLib passes a void(void) callback; gpio_isr_handler expects
        // void(void*).  We cast here — the ISR ignores the arg parameter.
        gpio_isr_handler_add((gpio_num_t)interruptNum,
                             (gpio_isr_t)interruptCb, NULL);
        gpio_intr_enable((gpio_num_t)interruptNum);
    }

    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum == RADIOLIB_NC) return;
        gpio_isr_handler_remove((gpio_num_t)interruptNum);
        gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    // ── Timing ──────────────────────────────────────────────

    void delay(unsigned long ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void delayMicroseconds(unsigned long us) override {
        ets_delay_us(us);
    }

    unsigned long millis() override {
        return (unsigned long)(esp_timer_get_time() / 1000ULL);
    }

    unsigned long micros() override {
        return (unsigned long)(esp_timer_get_time());
    }

    long pulseIn(uint32_t pin, uint32_t state,
                 unsigned long timeout) override {
        if (pin == RADIOLIB_NC) return 0;

        this->pinMode(pin, ESPHAL_INPUT);
        uint32_t start = this->micros();
        uint32_t curtick = this->micros();

        while (this->digitalRead(pin) == state) {
            if ((this->micros() - curtick) > timeout) {
                return 0;
            }
        }

        return (long)(this->micros() - start);
    }

    // ── SPI (using ESP-IDF spi_master driver) ───────────────

    void spiBegin() {
        spi_bus_config_t busCfg = {};
        busCfg.mosi_io_num = spiMOSI;
        busCfg.miso_io_num = spiMISO;
        busCfg.sclk_io_num = spiSCK;
        busCfg.quadwp_io_num = -1;
        busCfg.quadhd_io_num = -1;
        busCfg.max_transfer_sz = 256;

        esp_err_t ret = spi_bus_initialize(spiHostDev, &busCfg,
                                           SPI_DMA_CH_AUTO);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            // ESP_ERR_INVALID_STATE means bus already initialized (shared bus)
            ESP_LOGE("EspHal", "SPI bus init failed: %s", esp_err_to_name(ret));
            return;
        }
        busInitialized = (ret == ESP_OK);

        spi_device_interface_config_t devCfg = {};
        devCfg.mode = 0;                  // SPI mode 0 (CPOL=0, CPHA=0)
        devCfg.clock_speed_hz = spiClockHz;
        devCfg.spics_io_num = -1;         // CS handled by RadioLib Module
        devCfg.queue_size = 1;
        devCfg.flags = 0;
        devCfg.pre_cb = NULL;
        devCfg.post_cb = NULL;

        ret = spi_bus_add_device(spiHostDev, &devCfg, &spiDevice);
        if (ret != ESP_OK) {
            ESP_LOGE("EspHal", "SPI add device failed: %s",
                     esp_err_to_name(ret));
            spiDevice = NULL;
        }
    }

    void spiBeginTransaction() {
        // Acquire the bus for this device (blocks until available).
        // This is a no-op when only one device is on the bus.
        if (spiDevice) {
            spi_device_acquire_bus(spiDevice, portMAX_DELAY);
        }
    }

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
        if (!spiDevice || len == 0) return;

        spi_transaction_t t = {};
        t.length = len * 8;                // length in bits
        t.tx_buffer = out;
        t.rx_buffer = in;

        // Polling-mode (busy-wait, no internal ret_queue) avoids the
        // `assert(ret_trans == trans_desc)` panic in spi_device_transmit
        // that fires when the IDF v5.3.2 SPI master driver's per-device
        // ret_queue gets corrupted by cross-task internal state. SX1262
        // transfers are small (max 256 B per max_transfer_sz at 2 MHz)
        // so the CPU spin is negligible. See issue #115.
        spi_device_polling_transmit(spiDevice, &t);
    }

    void spiEndTransaction() {
        if (spiDevice) {
            spi_device_release_bus(spiDevice);
        }
    }

    void spiEnd() {
        if (spiDevice) {
            spi_bus_remove_device(spiDevice);
            spiDevice = NULL;
        }
        if (busInitialized) {
            spi_bus_free(spiHostDev);
            busInitialized = false;
        }
    }

private:
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    spi_host_device_t spiHostDev;
    uint32_t spiClockHz;
    spi_device_handle_t spiDevice = NULL;
    bool busInitialized = false;
};
