// TR_GNSSReceiverSdr_Serial.cpp — see TR_GNSSReceiverSdr_Serial.h.
#include "TR_GNSSReceiverSdr_Serial.h"

#include <algorithm>
#include <cstring>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>

static const char* TAG = "GNSS";

// 16 kB holds ~0.3 s of the stream at 55 kB/s; the poll task reads it every
// 25 ms (~1.4 kB). The UART interrupt is in IRAM on every image that has this
// driver (sdkconfig.defaults.mosaic, checked by gnss_auto_driver.cpp): at
// 2 Mbaud the 128-byte FIFO lasts 0.64 ms, less than a flash write.
static constexpr size_t kUartRxBuf = 16384;
static constexpr size_t kUartTxBuf = 256;   // nothing is ever sent
// Interrupt at half the FIFO: 320 us of margin before a late one loses bytes.
static constexpr int kRxFullThreshold = 64;

// Per pollNewPVT() call: twice the ring's worth of a slow poll's share, so a
// backlog after a stalled poll drains within a few calls. Framing it is a
// CRC per byte, a few hundred microseconds.
static constexpr size_t kMaxBytesPerPoll = 8192U;

// 8N1 at 2 Mbaud: ten bits a byte.
static constexpr double kByteUs = 1.0e7 / static_cast<double>(sdr_rtcm::kBaud);

static constexpr uint32_t kWeekMs = 604800000U;

static inline int64_t nowUs() { return esp_timer_get_time(); }

TR_GNSSReceiverSdrSerial::TR_GNSSReceiverSdrSerial(uart_port_t uart_port)
    : _uartPort(uart_port)
{
}

bool TR_GNSSReceiverSdrSerial::begin(uint8_t update_rate_hz,
                                     uint8_t GNSS_RX,
                                     uint8_t GNSS_TX,
                                     int8_t reset_n_pin,
                                     int8_t safeboot_n_pin)
{
    (void)update_rate_hz;
    (void)reset_n_pin;
    (void)safeboot_n_pin;
    rx_pin_ = GNSS_RX;
    tx_pin_ = GNSS_TX;

    if (ring_ == nullptr)
    {
        // Checked here, not trusted: a misaligned ring passes creation and
        // then asserts on the first send (see ring_mem_).
        if ((reinterpret_cast<uintptr_t>(ring_mem_) & 0x3U) != 0)
        {
            ESP_LOGE(TAG, "gnss-sdr-dev: the log ring is not 32-bit aligned; its frames will not be logged");
        }
        else
        {
            ring_ = xRingbufferCreateStatic(kRingBytes, RINGBUF_TYPE_NOSPLIT, ring_mem_, &ring_struct_);
        }
    }

    if (!uart_is_driver_installed(_uartPort))
    {
        uart_config_t cfg = {};
        cfg.baud_rate  = static_cast<int>(sdr_rtcm::kBaud);
        cfg.data_bits  = UART_DATA_8_BITS;
        cfg.parity     = UART_PARITY_DISABLE;
        cfg.stop_bits  = UART_STOP_BITS_1;
        cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
        cfg.source_clk = UART_SCLK_DEFAULT;
        esp_err_t e = uart_param_config(_uartPort, &cfg);
        if (e == ESP_OK)
        {
            e = uart_set_pin(_uartPort, static_cast<int>(tx_pin_), static_cast<int>(rx_pin_),
                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        }
        if (e == ESP_OK)
        {
            // Idle-high while nothing drives RX, as the other GNSS drivers do.
            gpio_pullup_en(static_cast<gpio_num_t>(rx_pin_));
            e = uart_driver_install(_uartPort, kUartRxBuf, kUartTxBuf, 16, &uart_events_, 0);
        }
        if (e != ESP_OK)
        {
            ESP_LOGE(TAG, "gnss-sdr-dev UART: setup at %lu baud failed (%s)",
                     static_cast<unsigned long>(sdr_rtcm::kBaud), esp_err_to_name(e));
            uart_events_ = nullptr;
            return false;
        }
        uart_set_rx_full_threshold(_uartPort, kRxFullThreshold);
    }
    else if (uart_set_baudrate(_uartPort, sdr_rtcm::kBaud) != ESP_OK)
    {
        return false;
    }
    uart_flush_input(_uartPort);
    framer_.reset();
    status_us_ = nowUs();
    ESP_LOGI(TAG, "gnss-sdr-dev: listening for its RTCM3 at %lu baud (RX=%u); logged, not navigated on",
             static_cast<unsigned long>(sdr_rtcm::kBaud), rx_pin_);
    return true;
}

// ── The UART ────────────────────────────────────────────────────────────

void TR_GNSSReceiverSdrSerial::drainUartEvents()
{
    if (uart_events_ == nullptr) return;
    uart_event_t ev;
    while (xQueueReceive(uart_events_, &ev, 0) == pdTRUE)
    {
        // The driver has already reset the FIFO or stopped reading into a full
        // ring; the bytes are gone and the framer's CRC drops the frame they
        // were in. Counted for the bench.
        if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) fifo_overflows_++;
    }
}

void TR_GNSSReceiverSdrSerial::pump(size_t max_bytes)
{
    size_t left = max_bytes;
    while (left > 0 && uart_is_driver_installed(_uartPort))
    {
        const size_t want = std::min(left, sizeof(rx_buf_));
        const int n = uart_read_bytes(_uartPort, rx_buf_, want, 0);
        if (n <= 0) return;
        // When each byte arrived: the read time less the bytes after it, the
        // rest of this read and what is still buffered, at one byte time each
        // (the LC86G and mosaic drivers' rule: exact while the board is still
        // sending, late by the gap otherwise, never early).
        size_t behind = 0;
        uart_get_buffered_data_len(_uartPort, &behind);
        rd_us_ = nowUs();
        rd_behind_ = behind;
        framer_.feed(rx_buf_, static_cast<size_t>(n), *this);
        left -= static_cast<size_t>(n);
    }
}

bool TR_GNSSReceiverSdrSerial::pollNewPVT(GNSSData& data)
{
    (void)data;
    pump(kMaxBytesPerPoll);
    drainUartEvents();
    logStatus();
    return false;
}

// ── Frames ──────────────────────────────────────────────────────────────

void TR_GNSSReceiverSdrSerial::onFrame(const uint8_t* frame, size_t len, size_t after)
{
    const int64_t end_us = rd_us_ - static_cast<int64_t>(static_cast<double>(after + rd_behind_) * kByteUs);

    // The flight log's copy: every frame, verbatim. Never blocks; a full ring
    // means nobody is draining it. Before the flight loop runs that is
    // expected (the boot), so only drops after it starts are counted.
    if (ring_ != nullptr && len <= sdr_rtcm::kMaxFrame)
    {
        const uint32_t t = static_cast<uint32_t>(end_us);
        std::memcpy(item_, &t, 4);
        std::memcpy(item_ + 4, frame, len);
        if (xRingbufferSend(ring_, item_, 4 + len, 0) != pdTRUE && draining_.load(std::memory_order_relaxed))
        {
            ring_dropped_++;
        }
    }

    const uint16_t number = sdr_rtcm::messageNumber(frame, len);
    switch (number)
    {
        case 1077:
        case 1097:
        case 1117:
        case 1127:
            n_msm_++;
            break;
        case 1019:
        case 1042:
        case 1046:
            n_eph_++;
            break;
        case sdr_rtcm::kTrMessage:
        {
            tr_frames_.fetch_add(1, std::memory_order_relaxed);
            sdr_rtcm::TrEpoch e;
            sdr_rtcm::TrFix f;
            if (sdr_rtcm::decodeTrEpoch(frame, len, e))
            {
                noteEpoch(e);
            }
            else if (sdr_rtcm::decodeTrFix(frame, len, f))
            {
                n_fixes_++;
                n_fix_valid_ += f.valid ? 1 : 0;
                last_fix_valid_ = f.valid;
                last_fix_nsat_ = f.nsat;
            }
            break;
        }
        default:
            n_other_++;
            break;
    }
}

void TR_GNSSReceiverSdrSerial::noteEpoch(const sdr_rtcm::TrEpoch& e)
{
    n_epochs_++;
    last_nobs_ = e.nobs;
    const uint8_t mode = e.mode & 0x03;
    if (mode == last_mode_)
    {
        const uint32_t step = (e.tow_ms + kWeekMs - last_tow_ms_) % kWeekMs;
        if (step == 0 || step > kWeekMs / 2)
        {
            epoch_backsteps_++;
        }
        else
        {
            // The mode's step is the shortest seen; anything longer is
            // epochs that never came.
            uint32_t& s = step_ms_[mode];
            if (s == 0 || step < s) s = step;
            if (step > s) epoch_gaps_ += step / s - 1 + ((step % s) != 0 ? 1 : 0);
        }
    }
    last_mode_ = mode;
    last_tow_ms_ = e.tow_ms;
}

bool TR_GNSSReceiverSdrSerial::pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us)
{
    if (ring_ == nullptr) return false;
    draining_.store(true, std::memory_order_relaxed);
    size_t item_len = 0;
    uint8_t* item = static_cast<uint8_t*>(xRingbufferReceive(ring_, &item_len, 0));
    if (item == nullptr) return false;
    const bool fits = item_len > 4 && item_len - 4 <= cap;
    if (fits)
    {
        std::memcpy(&time_us, item, 4);
        len = item_len - 4;
        std::memcpy(out, item + 4, len);
    }
    vRingbufferReturnItem(ring_, item);
    if (!fits) short_buffer_++;   // the caller's buffer is shorter than a frame
    return fits;
}

void TR_GNSSReceiverSdrSerial::logStatus()
{
    // Every 30 s on the boot console: what a bench run needs to see.
    const int64_t now = nowUs();
    const double span_s = static_cast<double>(now - status_us_) * 1.0e-6;
    if (span_s < 30.0) return;
    status_us_ = now;
    const uint32_t frames = framer_.frames();
    const double fps = (frames - status_frames_) / span_s;
    const double eps = (n_epochs_ - status_epochs_) / span_s;
    status_frames_ = frames;
    status_epochs_ = n_epochs_;

    if (frames == 0)
    {
        ESP_LOGI(TAG, "gnss-sdr-dev: nothing heard yet (it sends from its first fix; %lu bytes in no frame, "
                      "UART overflows %lu)",
                 static_cast<unsigned long>(framer_.skipped()), static_cast<unsigned long>(fifo_overflows_));
        return;
    }
    ESP_LOGI(TAG, "gnss-sdr-dev: %.1f frames/s, %.1f epochs/s (%u observables, epoch gaps %lu, back-steps %lu); "
                  "%lu fixes (%lu valid, last %s %u sats); %lu MSM7, %lu ephemerides, %lu other; "
                  "CRC fails %lu, bytes in no frame %lu, UART overflows %lu, log ring drops %lu; "
                  "poll task stack %lu B free",
             fps, eps, static_cast<unsigned>(last_nobs_), static_cast<unsigned long>(epoch_gaps_),
             static_cast<unsigned long>(epoch_backsteps_), static_cast<unsigned long>(n_fixes_),
             static_cast<unsigned long>(n_fix_valid_), last_fix_valid_ ? "valid" : "none,",
             static_cast<unsigned>(last_fix_nsat_), static_cast<unsigned long>(n_msm_),
             static_cast<unsigned long>(n_eph_), static_cast<unsigned long>(n_other_),
             static_cast<unsigned long>(framer_.crcFails()), static_cast<unsigned long>(framer_.skipped()),
             static_cast<unsigned long>(fifo_overflows_), static_cast<unsigned long>(ring_dropped_),
             static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
}
