// TR_GNSSReceiverSdr_Serial.cpp — see TR_GNSSReceiverSdr_Serial.h.
#include "TR_GNSSReceiverSdr_Serial.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <GnssClockRecord.h>   // TR_GNSSReceiverLC86_Serial: GNSS_CLOCK_MSG's record
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
#include <new>
#include <sdkconfig.h>
#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>   // xQueueCreateWithCaps
#include "SdrRawFeed.h"
// The board streams up to ~55 kB/s at 2 Mbaud, which fills the UART's
// 128-byte RX FIFO in 0.64 ms: less than a flash write keeps an interrupt
// that needs the cache waiting. sdkconfig.defaults.mosaic puts the UART
// interrupt in IRAM (as the auto image, gnss_auto_driver.cpp); a build dir
// made before this image had the overlay keeps its old sdkconfig.
#if !CONFIG_UART_ISR_IN_IRAM
#error "TR_GNSS_DRIVER_SDR needs CONFIG_UART_ISR_IN_IRAM (sdkconfig.defaults.mosaic): use a fresh build dir, or delete its sdkconfig"
#endif
#endif

static const char* TAG = "GNSS";

// 16 kB holds ~0.3 s of the stream at 55 kB/s; the poll task reads it every
// 25 ms (~1.4 kB). The UART interrupt is in IRAM on every image that has this
// driver (sdkconfig.defaults.mosaic, checked by gnss_auto_driver.cpp): at
// 2 Mbaud the 128-byte FIFO lasts 0.64 ms, less than a flash write.
static constexpr size_t kUartRxBuf = 16384;
// What goes out: 4002 frames, one of 52 B at most a millisecond (SdrAid.h
// AidSchedule; a flight image's phase, 22 B, about once a second), so the
// ring is all but empty at every write. sendAidFrame checks its room first;
// kTxItemSlack is a margin over what the driver says is free (it already
// takes off its own item header).
static constexpr size_t kUartTxBuf = 256;
static constexpr size_t kTxItemSlack = 32;
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

#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
// The raw outputs (enableRawOutputs). Three epochs (~2 kB each, internal
// RAM): the GNSS task takes them every 5 ms, and two more ride out a pass it
// spends on something else. 40 ephemerides in PSRAM: the board sends each one
// it holds again every 30 s, about 30 at once; 16 where there is no PSRAM
// (what one burst drops, the next 30 s brings).
static constexpr UBaseType_t kRawEpochQDepth  = 3;
static constexpr UBaseType_t kEphQDepthPsram  = 40;
static constexpr UBaseType_t kEphQDepthInternal = 16;
#endif

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
    if (clock_q_ == nullptr)
    {
        clock_q_ = xQueueCreateStatic(kClockQDepth, sizeof(GnssClockData), clock_q_mem_, &clock_q_struct_);
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
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
    ESP_LOGI(TAG, "gnss-sdr-dev: listening for its RTCM3 at %lu baud (RX=%u); logged, and %s",
             static_cast<unsigned long>(sdr_rtcm::kBaud), rx_pin_,
             raw_ != nullptr ? "navigated on by the raw-GNSS filter (its epochs, ephemerides and fixes)"
                             : "its fixes only (no raw outputs this boot)");
#else
    ESP_LOGI(TAG, "gnss-sdr-dev: listening for its RTCM3 at %lu baud (RX=%u); logged, not navigated on",
             static_cast<unsigned long>(sdr_rtcm::kBaud), rx_pin_);
#endif
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
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
    pump(kMaxBytesPerPoll);
    drainUartEvents();
    logStatus();
    if (!new_fix_) return false;   // `data` untouched (the u-blox driver's contract)
    new_fix_ = false;
    data = fix_data_;
    return true;
#else
    (void)data;
    pump(kMaxBytesPerPoll);
    drainUartEvents();
    logStatus();
    return false;
#endif
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
                noteLink(frame, len, e, end_us);
            }
            else if (sdr_rtcm::decodeTrFix(frame, len, f))
            {
                n_fixes_++;
                n_fix_valid_ += f.valid ? 1 : 0;
                last_fix_valid_ = f.valid;
                last_fix_nsat_ = f.nsat;
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
                noteFix(f, end_us);
#endif
            }
            else if (sdr_rtcm::decodeTrStatus(frame, len, last_status_))
            {
                n_status_++;
            }
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
            else
            {
                sdr_rtcm::TrIono io;
                if (sdr_rtcm::decodeTrIono(frame, len, io))
                {
                    portENTER_CRITICAL(&iono_mux_);
                    iono_ = io;
                    iono_have_ = true;
                    n_iono_++;
                    portEXIT_CRITICAL(&iono_mux_);
                }
            }
#endif
            break;
        }
        default:
            n_other_++;
            break;
    }
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
    // The raw outputs, after the clock line took this record's sample
    // (noteLink): the epoch it makes whole is placed on the line as it now
    // stands (SdrRawFeed.h has the order)
    if (raw_ != nullptr) rawFrame(frame, len, number, end_us);
#endif
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

void TR_GNSSReceiverSdrSerial::noteLink(const uint8_t* frame, size_t len, const sdr_rtcm::TrEpoch& e, int64_t end_us)
{
    sdr_aid::EpochAid aid;
    if (!sdr_aid::decodeEpochAid(frame, len, aid)) return;

    // The clock line, on every image (a flight image stamps its phase frames
    // off it, a bench image its script's): each epoch at its latch against
    // when its 4001 ended here, but those the board marks late
    // (sdr_aid::latchGps). The board's delay from the latch to that frame
    // stays in (latency_s 0).
    double latch_gps = 0.0;
    if (sdr_aid::latchGps(e, aid, latch_gps))
    {
        time_sync_.addSample(latch_gps, end_us);
    }
    else
    {
        late_left_out_++;
    }
    // The line each time it moves, from this task, to the flight loop
    if (clock_q_ != nullptr && time_sync_.lineSeq() != clock_seq_seen_)
    {
        clock_seq_seen_ = time_sync_.lineSeq();
        lc86::GnssTimeSync::Snapshot snap;
        time_sync_.snapshot(snap);
        GnssClockData rec;
        lc86::gnssClockRecord(snap, static_cast<uint32_t>(end_us), rec);
        if (xQueueSend(clock_q_, &rec, 0) != pdTRUE) clock_dropped_++;
    }

#if defined(TR_BENCH_SDR_AID) && TR_BENCH_SDR_AID
    // What the board says it took, against what went out (the bench's link
    // line)
    if (!aid.present) return;
    uint32_t sent = 0;
    for (uint8_t sub = sdr_aid::kAttitude; sub <= sdr_aid::kGeometry; sub++)
        sent += aid_sent_[sub].load(std::memory_order_relaxed);
    echo_.add(aid.fc_frames, sent, end_us);
    last_aid_ = aid;
    n_aid_++;
#endif
}

// ── Raw outputs for the raw-GNSS navigation filter ──────────────────────
// The SDR raw-nav image's alone (TR_GNSS_DRIVER_SDR; the header says why).
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR

bool TR_GNSSReceiverSdrSerial::enableRawOutputs()
{
    if (raw_ != nullptr) return true;
    // Internal RAM: the poll task works every MSM7 through it, 300 a second.
    // PSRAM where internal RAM has not the room (the cache keeps it close).
    bool psram = false;
    void* mem = heap_caps_malloc(sizeof(RawState), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (mem == nullptr)
    {
        psram = true;
        mem = heap_caps_malloc(sizeof(RawState), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (mem == nullptr)
    {
        ESP_LOGE(TAG, "gnss-sdr-dev raw: no memory for the raw outputs (%u B)", static_cast<unsigned>(sizeof(RawState)));
        return false;
    }
    RawState* r = new (mem) RawState();
    r->epoch_q = xQueueCreate(kRawEpochQDepth, sizeof(gnss_raw::TimedRawEpoch));
    r->eph_q = xQueueCreateWithCaps(kEphQDepthPsram, sizeof(EphItem), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (r->eph_q != nullptr)
    {
        r->eph_caps = true;
        r->st.eph_in_psram = true;
        r->st.eph_depth = static_cast<uint16_t>(kEphQDepthPsram);
    }
    else
    {
        r->eph_q = xQueueCreate(kEphQDepthInternal, sizeof(EphItem));
        r->st.eph_depth = static_cast<uint16_t>(kEphQDepthInternal);
    }
    if (r->epoch_q == nullptr || r->eph_q == nullptr)
    {
        // nothing half made: give back what was got
        ESP_LOGE(TAG, "gnss-sdr-dev raw: no memory for the epoch queue (%u B) or the ephemeris queue",
                 static_cast<unsigned>(kRawEpochQDepth * sizeof(gnss_raw::TimedRawEpoch)));
        if (r->epoch_q != nullptr) vQueueDelete(r->epoch_q);
        if (r->eph_q != nullptr)
        {
            if (r->eph_caps) vQueueDeleteWithCaps(r->eph_q);
            else vQueueDelete(r->eph_q);
        }
        r->~RawState();
        heap_caps_free(mem);
        return false;
    }
    r->st.enabled = true;
    r->st.in_psram = psram;
    raw_ = r;   // before begin(): the poll task does not exist yet
    ESP_LOGI(TAG, "gnss-sdr-dev raw: outputs on -- %u epochs (%u B, internal), %u ephemerides in %s, %u B of state "
                  "in %s",
             static_cast<unsigned>(kRawEpochQDepth),
             static_cast<unsigned>(kRawEpochQDepth * sizeof(gnss_raw::TimedRawEpoch)),
             static_cast<unsigned>(r->st.eph_depth), r->st.eph_in_psram ? "PSRAM" : "internal RAM",
             static_cast<unsigned>(sizeof(RawState)), psram ? "PSRAM" : "internal RAM");
    return true;
}

void TR_GNSSReceiverSdrSerial::setRawEpochConfig(uint32_t fuse_period_ms, const sdr_rtcm::SdrDopplerMeanConfig& mean)
{
    if (raw_ == nullptr || raw_->pending_set.load(std::memory_order_acquire)) return;
    raw_->pending.period_ms = fuse_period_ms;
    raw_->pending.mean = mean;
    raw_->pending_set.store(true, std::memory_order_release);   // the poll task takes it at its next frame
}

bool TR_GNSSReceiverSdrSerial::takeRawEpoch(gnss_raw::TimedRawEpoch& out)
{
    if (raw_ == nullptr) return false;
    raw_->taking_epochs.store(true, std::memory_order_relaxed);
    return xQueueReceive(raw_->epoch_q, &out, 0) == pdTRUE;
}

bool TR_GNSSReceiverSdrSerial::broadcastIono(int8_t raw[8], uint16_t& gps_day, uint8_t* src) const
{
    portENTER_CRITICAL(&iono_mux_);
    const bool have = iono_have_;
    if (have)
    {
        std::memcpy(raw, iono_.raw, sizeof(iono_.raw));
        gps_day = iono_.gpsDay();
        if (src != nullptr) *src = iono_.src;
    }
    portEXIT_CRITICAL(&iono_mux_);
    return have;
}

bool TR_GNSSReceiverSdrSerial::takeEphemeris(uint8_t* out, size_t cap, size_t& len)
{
    if (raw_ == nullptr) return false;
    raw_->taking_eph.store(true, std::memory_order_relaxed);
    EphItem it;
    while (xQueueReceive(raw_->eph_q, &it, 0) == pdTRUE)
    {
        if (it.len > cap)
        {
            raw_->st.eph_short_buffer++;   // this task's own counter
            continue;
        }
        std::memcpy(out, it.bytes, it.len);
        len = it.len;
        return true;
    }
    return false;
}

void TR_GNSSReceiverSdrSerial::rawStats(RawStats& out) const
{
    out = (raw_ != nullptr) ? raw_->st : RawStats{};
}

void TR_GNSSReceiverSdrSerial::noteFix(const sdr_rtcm::TrFix& f, int64_t end_us)
{
    // The board's fix as GNSSData (SdrRawFeed.h): handed over by pollNewPVT
    // when it is one the flight computer can use
    GNSSData d;
    if (!sdr_rtcm::gnssDataFromTrFix(f, static_cast<uint32_t>(end_us), d)) return;
    fix_data_ = d;
    new_fix_ = true;
    if (raw_ != nullptr) raw_->st.fixes++;
}

void TR_GNSSReceiverSdrSerial::rawFrame(const uint8_t* frame, size_t len, uint16_t number, int64_t end_us)
{
    RawState& r = *raw_;
    if (r.pending_set.load(std::memory_order_acquire))
    {
        // setRawEpochConfig's: the grid and the Doppler window start over
        sdr_rtcm::SdrEpochAssembler::Config c = r.assembler.config();
        c.fuse_period_ms = r.pending.period_ms;
        c.mean = r.pending.mean;
        r.assembler.setConfig(c);
        r.st.period_ms = c.fuse_period_ms;
        r.st.mean_n = r.assembler.config().mean.n >= 2 ? r.assembler.config().mean.n : 0;
        r.pending_set.store(false, std::memory_order_release);
    }
    if (number == 1019 || number == 1042 || number == 1046)
    {
        // its payload, as the LC86G driver queues its ephemerides
        const size_t n = len >= 6 ? len - 6 : 0;
        if (n == 0 || n > kEphMaxPayload)
        {
            r.st.eph_too_long++;
            return;
        }
        r.eph_item.len = static_cast<uint8_t>(n);
        std::memcpy(r.eph_item.bytes, frame + 3, n);
        if (xQueueSend(r.eph_q, &r.eph_item, 0) == pdTRUE)
            r.st.eph_queued++;
        else if (r.taking_eph.load(std::memory_order_relaxed))
            r.st.eph_dropped++;   // until its next repeat, 30 s on
        return;
    }
    const bool msm = number == 1077 || number == 1097 || number == 1117 || number == 1127;
    if (!msm && number != sdr_rtcm::kTrMessage) return;
    const int64_t t0 = nowUs();
    const bool handed = r.assembler.onFrame(frame, len, end_us, r.epoch);
    if (handed)
    {
        (void)sdr_rtcm::timedEpochOf(r.epoch, time_sync_, r.timed);
        if (xQueueSend(r.epoch_q, &r.timed, 0) == pdTRUE)
        {
            r.st.published++;
            if (r.timed.mcu_us == 0) r.st.untimed++;
            if (r.timed.rr_paired) r.st.with_means++;
        }
        else if (r.taking_epochs.load(std::memory_order_relaxed))
        {
            r.st.dropped++;   // full: the GNSS task is not taking them (yet, at boot: not counted)
        }
    }
    const uint32_t us = static_cast<uint32_t>(nowUs() - t0);
    if (msm)
    {
        r.st.msm_n++;
        r.st.msm_us_sum += us;
        if (us > r.st.msm_us_max) r.st.msm_us_max = us;
    }
    else if (sdr_rtcm::trSubType(frame, len) == sdr_rtcm::kTrEpoch)
    {
        r.st.rec_n++;
        r.st.rec_us_sum += us;
        if (us > r.st.rec_us_max) r.st.rec_us_max = us;
    }
    r.st.asm_counts = r.assembler.counts();
    r.st.mean = r.assembler.meanStats();
}

void TR_GNSSReceiverSdrSerial::logRaw()
{
    if (raw_ == nullptr) return;
    // A second line after the status line, every 30 s: what the raw-GNSS
    // filter was handed and what it cost this task
    const RawStats& st = raw_->st;
    const sdr_rtcm::SdrEpochAssembler::Counts& c = st.asm_counts;
    const sdr_rtcm::SdrDopplerMeanStats& m = st.mean;
    sdr_rtcm::TrIono io;
    portENTER_CRITICAL(&iono_mux_);
    const bool have_io = iono_have_;
    const uint32_t n_io = n_iono_;
    io = iono_;
    portEXIT_CRITICAL(&iono_mux_);
    ESP_LOGI(TAG, "gnss-sdr-dev raw: grid %lu ms, Doppler means of %lu | epochs %lu whole (%lu off the grid, %lu "
                  "without their record, %lu records alone), %lu handed over (%lu untimed, %lu with means, %lu "
                  "queue drops) | means: %lu of %lu Dopplers, epochs as they came %lu short / %lu few, dropped %lu "
                  "missing %lu relock %lu step (largest step %.2f, spread %.2f m/s) | marked out %lu pr %lu Doppler | "
                  "eph %lu queued, %lu drops | fixes %lu | iono %s (%d %d %d %d / %d %d %d %d, GPS day %u; %lu frames) | "
                  "poll task: MSM7 %.0f us mean, %lu max; record %.0f us "
                  "mean, %lu max",
             static_cast<unsigned long>(st.period_ms), static_cast<unsigned long>(st.mean_n),
             static_cast<unsigned long>(c.epochs), static_cast<unsigned long>(c.off_grid),
             static_cast<unsigned long>(c.no_record), static_cast<unsigned long>(c.lone_record),
             static_cast<unsigned long>(st.published), static_cast<unsigned long>(st.untimed),
             static_cast<unsigned long>(st.with_means), static_cast<unsigned long>(st.dropped),
             static_cast<unsigned long>(m.means), static_cast<unsigned long>(m.dopplers),
             static_cast<unsigned long>(m.epochs_short), static_cast<unsigned long>(m.epochs_few),
             static_cast<unsigned long>(m.alone_missing), static_cast<unsigned long>(m.alone_lock),
             static_cast<unsigned long>(m.alone_step), static_cast<double>(m.step_max_mps),
             static_cast<double>(m.spread_max_mps), static_cast<unsigned long>(c.marked_pr),
             static_cast<unsigned long>(c.marked_rate), static_cast<unsigned long>(st.eph_queued),
             static_cast<unsigned long>(st.eph_dropped), static_cast<unsigned long>(st.fixes),
             !have_io ? "NONE" : (io.src == sdr_rtcm::kIonoLnav ? "decoded" : "the loaded file's"), io.raw[0],
             io.raw[1], io.raw[2], io.raw[3], io.raw[4], io.raw[5], io.raw[6], io.raw[7],
             static_cast<unsigned>(have_io ? io.gpsDay() : 0), static_cast<unsigned long>(n_io),
             st.msm_n ? static_cast<double>(st.msm_us_sum) / static_cast<double>(st.msm_n) : 0.0,
             static_cast<unsigned long>(st.msm_us_max),
             st.rec_n ? static_cast<double>(st.rec_us_sum) / static_cast<double>(st.rec_n) : 0.0,
             static_cast<unsigned long>(st.rec_us_max));
}

#endif  // TR_GNSS_DRIVER_SDR

// ── To the board ────────────────────────────────────────────────────────

bool TR_GNSSReceiverSdrSerial::sendAidFrame(const uint8_t* frame, size_t len)
{
    // uart_write_bytes waits on portMAX_DELAY for the TX ring and spins while
    // it is full (IDF uart.c uart_tx_all), so its room is checked first, by
    // its one writer (the poll task only reads, under the driver's other
    // lock). Never uart_wait_tx_done.
    if (frame == nullptr || len < 6 || len > sdr_aid::kMaxFrame)
    {
        aid_bad_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (!uart_is_driver_installed(_uartPort))
    {
        aid_no_uart_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    size_t room = 0;
    if (uart_get_tx_buffer_free_size(_uartPort, &room) != ESP_OK || room < len + kTxItemSlack)
    {
        aid_tx_full_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (uart_write_bytes(_uartPort, frame, len) != static_cast<int>(len))
    {
        aid_bad_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    aid_sent_[frame[4] & 0x0F].fetch_add(1, std::memory_order_relaxed);   // the sub-type
    return true;
}

bool TR_GNSSReceiverSdrSerial::takeClockLine(GnssClockData& out)
{
    return clock_q_ != nullptr && xQueueReceive(clock_q_, &out, 0) == pdTRUE;
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

    if (n_epochs_ == 0 && n_status_ > 0)
    {
        const sdr_rtcm::TrStatus& st = last_status_;
        ESP_LOGI(TAG, "gnss-sdr-dev: up, no GPS time yet (firmware %08lx%s, up %lu s, %u channels tracking, %u locked; "
                      "%lu status frames)",
                 static_cast<unsigned long>(st.fw), st.fw_dirty ? "-dirty" : "", static_cast<unsigned long>(st.uptime_s),
                 static_cast<unsigned>(st.ntrack), static_cast<unsigned>(st.nlock),
                 static_cast<unsigned long>(n_status_));
    }
    else if (frames == 0)
    {
        ESP_LOGI(TAG, "gnss-sdr-dev: nothing heard yet (a board with the status message sends one a second from its "
                      "receiver's start, older firmware only from its first fix; %lu bytes in no frame, UART "
                      "overflows %lu)",
                 static_cast<unsigned long>(framer_.skipped()), static_cast<unsigned long>(fifo_overflows_));
    }
    else
    {
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
#if defined(TR_BENCH_SDR_AID) && TR_BENCH_SDR_AID
    logLink();
#endif
#if defined(TR_GNSS_DRIVER_SDR) && TR_GNSS_DRIVER_SDR
    logRaw();
#endif
}

#if defined(TR_BENCH_SDR_AID) && TR_BENCH_SDR_AID
void TR_GNSSReceiverSdrSerial::logLink()
{
    // The bench's second line, after the first, every 30 s. It starts
    // "gnss-sdr-dev:" too, so the rig kit's grep for that shows both. The
    // counts are this line's 30 s. What the board took comes from the 8-bit
    // count in its epochs (sdr_aid::EchoCount), of what went out between
    // them: it lags what was sent by an epoch or so.
    uint32_t sent[sdr_aid::kGeometry + 1] = {};
    for (uint8_t sub = sdr_aid::kAttitude; sub <= sdr_aid::kGeometry; sub++)
    {
        const uint32_t n = aid_sent_[sub].load(std::memory_order_relaxed);
        sent[sub] = n - line_sent_[sub];
        line_sent_[sub] = n;
    }
    const uint32_t took = echo_.took() - line_took_;
    const uint32_t of = echo_.of() - line_of_;
    const uint32_t aside = echo_.setAside() - line_aside_;
    line_took_ = echo_.took();
    line_of_ = echo_.of();
    line_aside_ = echo_.setAside();

    char line_s[160];
    if (time_sync_.valid())
    {
        snprintf(line_s, sizeof(line_s), "valid (%.1f ppm, rms %.2f ms, 90 %% %.1f ms late, %lu restarts, "
                                         "%lu records lost, %lu late epochs left out)",
                 time_sync_.driftPpm(), time_sync_.envelopeRmsMs(), time_sync_.lateness90Ms(),
                 static_cast<unsigned long>(time_sync_.restarts()), static_cast<unsigned long>(clock_dropped_),
                 static_cast<unsigned long>(late_left_out_));
    }
    else
    {
        snprintf(line_s, sizeof(line_s), "not valid yet (%lu samples, %lu records lost, %lu late epochs left out)",
                 static_cast<unsigned long>(time_sync_.samples()), static_cast<unsigned long>(clock_dropped_),
                 static_cast<unsigned long>(late_left_out_));
    }

    char board_s[224];
    if (n_epochs_ == 0)
    {
        snprintf(board_s, sizeof(board_s), "no epochs from the board yet");
    }
    else if (n_aid_ == 0)
    {
        snprintf(board_s, sizeof(board_s), "the board's epochs carry no aiding block (firmware from before 4002)");
    }
    else
    {
        static const char* const kPhases[8] = {"none", "pad", "boost", "coast", "descent", "landed", "6", "7"};
        static const char* const kBy[4] = {"the GPS channels", "the flight computer", "a caller", "the board's IMU"};
        const sdr_aid::EpochAid& a = last_aid_;
        char aside_s[64] = "";
        if (aside > 0)
        {
            snprintf(aside_s, sizeof(aside_s), " (%lu more sent where its count could not be followed)",
                     static_cast<unsigned long>(aside));
        }
        snprintf(board_s, sizeof(board_s), "the board took %lu of %lu%s, holds the phase %s, its loops %s by %s, "
                                           "attitude %s, state %s",
                 static_cast<unsigned long>(took), static_cast<unsigned long>(of), aside_s, kPhases[a.phase & 7],
                 a.boost ? "boost" : "quiet", kBy[a.by & 3], a.att ? "yes" : "no", a.nav ? "yes" : "no");
    }

    ESP_LOGI(TAG, "gnss-sdr-dev: link out (bench script): sent %lu attitude, %lu state, %lu phase, %lu geometry "
                  "(TX full %lu, no UART %lu, bad writes %lu); time line %s; %s",
             static_cast<unsigned long>(sent[sdr_aid::kAttitude]), static_cast<unsigned long>(sent[sdr_aid::kNavState]),
             static_cast<unsigned long>(sent[sdr_aid::kPhase]), static_cast<unsigned long>(sent[sdr_aid::kGeometry]),
             static_cast<unsigned long>(aid_tx_full_.load(std::memory_order_relaxed)),
             static_cast<unsigned long>(aid_no_uart_.load(std::memory_order_relaxed)),
             static_cast<unsigned long>(aid_bad_.load(std::memory_order_relaxed)), line_s, board_s);
}
#endif
