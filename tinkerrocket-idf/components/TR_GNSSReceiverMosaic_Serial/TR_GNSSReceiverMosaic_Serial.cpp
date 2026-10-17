// TR_GNSSReceiverMosaic_Serial.cpp — Septentrio mosaic-G5 over ESP-IDF UART. See the header.
//
// References: mosaic-G5 Reference Guide, firmware 1.0.1 [RG]; the receiver
// itself (mosaic-G5 P3, firmware 1.0.0, permission file "Maximum Data Rate:
// Permitted up to 20 Hz"), as the 2026-10-15 spin-rig sessions logged it.
#include "TR_GNSSReceiverMosaic_Serial.h"

#include <GnssClockRecord.h>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

static const char* TAG = "GNSS";

// 2 Mbaud carries the full 20 Hz set (~66 kB/s measured) at a third of the
// line; 921600 would run at 71 %, with each epoch's burst filling 35 of its
// 50 ms. The G5's COM ports go to 4 Mbaud [RG scs]. The 1 kOhm the harness
// puts in each line leaves edges of a few tens of ns.
static constexpr uint32_t kTargetBaud  = 2000000U;
static constexpr uint32_t kFactoryBaud = 115200U;   // [RG scs]: every COM port's default

// The receiver shares the switched rail that powers up with the FC, so it is
// often still booting when begin() runs. Same order of budget as the u-blox
// driver's bring-up deadline; a healthy, already-set-up receiver answers in
// well under a second.
static constexpr uint32_t kBeginTimeoutMs = 30000U;   // the default; setBeginTimeoutMs() changes it

// 16 kB holds ~5 epochs of the full set; the poll task runs every 25 ms.
static constexpr size_t kUartRxBuf = 16384;
static constexpr size_t kUartTxBuf = 512;
// Interrupt at half the 128-byte FIFO: at 2 Mbaud the other half is 320 us
// of margin before a late interrupt loses bytes.
static constexpr int kRxFullThreshold = 64;

// Per pollNewPVT() call. The full set averages ~1.7 kB per 25 ms poll and an
// epoch arrives as one ~3.3 kB burst, so 4 kB keeps up with margin; parsing
// it (a CRC per byte and a copy per block) costs well under a millisecond.
static constexpr size_t kMaxBytesPerPoll = 4096U;

// PPS. Edges closer together than kPpsMinSpacingUs are glitches, not pulses;
// a burst of them mutes the interrupt for kPpsUnmuteUs, so a floating or
// noisy pin cannot flood core 0 with interrupts. Which second each edge was:
// PpsPairer.h.
static constexpr int64_t  kPpsMinSpacingUs = 200000;
static constexpr uint32_t kPpsBurstMute    = 20;
static constexpr int64_t  kPpsUnmuteUs     = 2000000;

// The blocks. The FC's streams are 5-10; the bench logger (mosaic_log.py)
// uses 1-4 on USB1, so both can run at once on the bench.
#define MOSAIC_FAST_BLOCKS   "MeasEpoch+MeasExtra+PVTGeodetic"
#define MOSAIC_STATUS_BLOCKS "ReceiverStatus+QualityInd+RFStatus+SatVisibility+ChannelStatus+DOP+ReceiverTime+DiskStatus"
#define MOSAIC_NAV_BLOCKS    "GPSNav+GPSCNav+GALNav+BDSNav+BDSCNav1+GLONav+QZSNav+GPSIon+GPSUtc+GALIon+GALUtc+GALGstGps+BDSIon+BDSUtc+GLOTime"
#define MOSAIC_CHANGE_BLOCKS MOSAIC_NAV_BLOCKS "+ReceiverSetup+Commands+RxMessage+xPPSOffset"

static const char* const kCmdStatusCom = "sso, Stream8, {cd}, " MOSAIC_STATUS_BLOCKS ", sec1";
static const char* const kCmdChangeCom = "sso, Stream9, {cd}, " MOSAIC_CHANGE_BLOCKS ", OnChange";
static const char* const kCmdStatusDsk = "sso, Stream6, DSK1, " MOSAIC_STATUS_BLOCKS ", sec1";
static const char* const kCmdChangeDsk = "sso, Stream10, DSK1, " MOSAIC_CHANGE_BLOCKS ", OnChange";
// After launch: the log starts there, and the receiver sent its navigation
// data when it first decoded it, long before (OnChange). [RG esoc]
static const char* const kCmdNavResend = "esoc, {cd}, " MOSAIC_NAV_BLOCKS "+ReceiverSetup";

static inline int64_t  nowUs() { return esp_timer_get_time(); }
static inline uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// The SBF interval for a rate: the receiver's own steps [RG sso].
static const char* intervalFor(uint8_t hz)
{
    if (hz >= 20) return "msec50";
    if (hz >= 10) return "msec100";
    if (hz >= 5)  return "msec200";
    if (hz >= 2)  return "msec500";
    return "sec1";
}

TR_GNSSReceiverMosaicSerial::TR_GNSSReceiverMosaicSerial(uart_port_t uart_port)
    : _uartPort(uart_port), begin_timeout_ms_(kBeginTimeoutMs) {}

// ── UART ────────────────────────────────────────────────────────────────

bool TR_GNSSReceiverMosaicSerial::uartBegin(uint32_t baud)
{
    // Installed once; after that a probe's baud change only re-rates the port.
    // The probe alternates bauds every 400 ms for as long as the receiver is
    // silent, and tearing the driver down and back up each time (its 16 kB
    // ring, its event queue, its IRAM interrupt) was churn for nothing. No
    // ESP_ERROR_CHECK: a port that will not come up means GNSS-absent, never
    // an abort in the middle of the boot.
    byte_us_ = 1.0e7 / static_cast<double>(baud);   // 8N1: ten bits a byte
    if (uart_is_driver_installed(_uartPort))
    {
        const esp_err_t e = uart_set_baudrate(_uartPort, baud);
        if (e != ESP_OK)
        {
            ESP_LOGE(TAG, "mosaic UART: baud %lu refused (%s)", static_cast<unsigned long>(baud), esp_err_to_name(e));
            return false;
        }
        uart_flush_input(_uartPort);   // bytes sampled at the old baud
        drainUartEvents();
        uart_baud_ = baud;
        return true;
    }

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = static_cast<int>(baud);
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t e = uart_param_config(_uartPort, &uart_cfg);
    if (e == ESP_OK)
    {
        e = uart_set_pin(_uartPort, static_cast<int>(tx_pin_), static_cast<int>(rx_pin_),
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (e == ESP_OK)
    {
        // Idle-high while nothing drives RX (a receiver still booting), as the
        // other GNSS drivers do.
        gpio_pullup_en(static_cast<gpio_num_t>(rx_pin_));
        e = uart_driver_install(_uartPort, kUartRxBuf, kUartTxBuf, 16, &uart_events_, 0);
    }
    if (e != ESP_OK)
    {
        ESP_LOGE(TAG, "mosaic UART: setup at %lu baud failed (%s)", static_cast<unsigned long>(baud),
                 esp_err_to_name(e));
        uart_events_ = nullptr;
        return false;
    }
    uart_set_rx_full_threshold(_uartPort, kRxFullThreshold);
    uart_baud_ = baud;
    return true;
}

void TR_GNSSReceiverMosaicSerial::uartEnd()
{
    if (uart_is_driver_installed(_uartPort))
    {
        uart_driver_delete(_uartPort);
    }
    uart_events_ = nullptr;
}

size_t TR_GNSSReceiverMosaicSerial::pump(uint32_t wait_ms, size_t max_bytes)
{
    if (!uart_is_driver_installed(_uartPort))
    {
        if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return 0;
    }
    const size_t want = std::min(max_bytes, sizeof(rx_buf_));
    const int n = uart_read_bytes(_uartPort, rx_buf_, want, pdMS_TO_TICKS(wait_ms));
    if (n <= 0) return 0;
    // When each byte arrived: the read time less the bytes after it, the rest
    // of this read and what is still buffered, at one byte time each (the
    // LC86G driver's rule: exact while the receiver is still sending, late
    // by the gap otherwise, never early). Taken BEFORE the time, so a byte
    // landing in between makes a stamp late, not early.
    size_t behind = 0;
    uart_get_buffered_data_len(_uartPort, &behind);
    rd_us_ = nowUs();
    rd_n_ = static_cast<size_t>(n);
    rd_behind_ = behind;
    parser_.feed(rx_buf_, rd_n_, *this);
    return rd_n_;
}

void TR_GNSSReceiverMosaicSerial::drainUartEvents()
{
    if (uart_events_ == nullptr) return;
    uart_event_t ev;
    while (xQueueReceive(uart_events_, &ev, 0) == pdTRUE)
    {
        // The driver has already reset the FIFO or stopped reading into a full
        // ring; the bytes are gone and the parser's CRC drops the block they
        // were in. Counted for the bench.
        if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) fifo_overflows_++;
    }
}

void TR_GNSSReceiverMosaicSerial::runLink()
{
    // A state queues at most a baud change and a send; the bound only guards.
    for (int i = 0; i < 8; i++)
    {
        const mosaic::LinkAction a = link_.poll(nowMs());
        if (a.kind == mosaic::LinkAction::NONE) break;
        if (a.kind == mosaic::LinkAction::SET_BAUD)
        {
            // On a failure the probe simply hears nothing at this baud and
            // moves on; the next change tries the port again.
            (void)uartBegin(a.baud);
            parser_.reset();   // bytes sampled at the old baud
        }
        else if (a.kind == mosaic::LinkAction::SEND && uart_is_driver_installed(_uartPort))
        {
            uart_write_bytes(_uartPort, a.text, std::strlen(a.text));
            // A command is a few hundred bytes at most (~1.5 ms at 2 Mbaud);
            // wait for the wire so a baud change can never cut one short.
            uart_wait_tx_done(_uartPort, pdMS_TO_TICKS(50));
        }
    }
    if (link_.setups() != setups_logged_)
    {
        setups_logged_ = link_.setups();
        logSetup();
    }
}

// ── begin() ─────────────────────────────────────────────────────────────

bool TR_GNSSReceiverMosaicSerial::begin(uint8_t update_rate_hz,
                                        uint8_t GNSS_RX,
                                        uint8_t GNSS_TX,
                                        int8_t reset_n_pin,
                                        int8_t safeboot_n_pin)
{
    // The V9's J1 has no reset or safe-boot line; the rail cycle is the reset.
    (void)reset_n_pin;
    (void)safeboot_n_pin;
    rx_pin_ = GNSS_RX;
    tx_pin_ = GNSS_TX;

    // The permission file allows 20 Hz, position and measurements alike.
    update_rate_hz_ = update_rate_hz;
    if (update_rate_hz_ < 1) update_rate_hz_ = 1;
    if (update_rate_hz_ > 20)
    {
        ESP_LOGW(TAG, "Requested %u Hz exceeds the mosaic-G5's permitted 20 Hz; using 20",
                 update_rate_hz);
        update_rate_hz_ = 20;
    }

    if (ring_ == nullptr)
    {
        // Checked here, not trusted: a misaligned ring passes creation and
        // then asserts on the first send (see ring_mem_).
        if ((reinterpret_cast<uintptr_t>(ring_mem_) & 0x3U) != 0)
        {
            ESP_LOGE(TAG, "mosaic: the log ring is not 32-bit aligned; SBF blocks will not be logged");
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

    // The setup, in order. Required: what the flight log and GNSSData need.
    // Not required: the receiver's own microSD log (no card, no harm).
    const char* iv = intervalFor(update_rate_hz_);
    std::snprintf(fast_com_, sizeof(fast_com_), "sso, Stream7, {cd}, " MOSAIC_FAST_BLOCKS ", %s", iv);
    std::snprintf(fast_dsk_, sizeof(fast_dsk_), "sso, Stream5, DSK1, " MOSAIC_FAST_BLOCKS ", %s", iv);
    n_cmds_ = 0;
    cmds_[n_cmds_++] = {"sdio, {cd}, CMD, SBF", true};          // commands in, SBF out, no NMEA
    cmds_[n_cmds_++] = {"srd, High, Unlimited", true};          // the most dynamic tracking/PVT (as on the rigs)
    cmds_[n_cmds_++] = {fast_com_, true};
    cmds_[n_cmds_++] = {kCmdStatusCom, true};
    cmds_[n_cmds_++] = {kCmdChangeCom, true};
    cmds_[n_cmds_++] = {"spps, sec1, Low2High, 0.00, GPS, 60, 5.000", true};   // 1PPS on GPS time, 5 ms
    cmds_[n_cmds_++] = {"sfn, DSK1, Incremental, trfc", false};  // a new trfc_NNN.sbf each power-up
    cmds_[n_cmds_++] = {fast_dsk_, false};
    cmds_[n_cmds_++] = {kCmdStatusDsk, false};
    cmds_[n_cmds_++] = {kCmdChangeDsk, false};
    cmds_[n_cmds_++] = {"sdio, DSK1, , +SBF", false};

    setupPps();

    mosaic::LinkConfig cfg;
    cfg.target_baud = kTargetBaud;
    cfg.factory_baud = kFactoryBaud;
    link_.start(cfg, cmds_, n_cmds_, nowMs());
    setups_logged_ = 0;

    ESP_LOGI(TAG, "Looking for the mosaic-G5 (RX=%u TX=%u PPS=%d), up to %lu s...",
             rx_pin_, tx_pin_, pps_pin_, static_cast<unsigned long>(begin_timeout_ms_ / 1000));
    const int64_t deadline_us = nowUs() + static_cast<int64_t>(begin_timeout_ms_) * 1000;
    while (nowUs() < deadline_us && !link_.running())
    {
        runLink();
        pump(20, sizeof(rx_buf_));
        runLink();
    }
    drainUartEvents();

    if (!link_.running())
    {
        ESP_LOGE(TAG, "GNSS bring-up: no mosaic-G5 answered in %lu s (last state %u); "
                      "continuing without GNSS",
                 static_cast<unsigned long>(begin_timeout_ms_ / 1000), static_cast<unsigned>(link_.state()));
        return false;
    }
    return true;
}

void TR_GNSSReceiverMosaicSerial::logSetup()
{
    ESP_LOGI(TAG, "mosaic-G5 set up: \"%s\" firmware %s on %s at %lu baud (setup %lu)",
             link_.product()[0] ? link_.product() : "?", link_.firmware()[0] ? link_.firmware() : "?",
             link_.port(), static_cast<unsigned long>(link_.baud()), static_cast<unsigned long>(link_.setups()));
    if (link_.refused() > 0)
    {
        ESP_LOGW(TAG, "mosaic-G5: %lu setup command(s) refused or unanswered, %lu of them required "
                      "(last: %s)", static_cast<unsigned long>(link_.refused()),
                 static_cast<unsigned long>(link_.requiredFailed()), link_.lastFailure());
    }
    if (link_.setups() == 1)
    {
        ESP_LOGI(TAG, "mosaic-G5: setup %s the receiver's boot configuration",
                 link_.saved() ? "saved into" : "NOT saved into");
    }
}

// ── PPS ─────────────────────────────────────────────────────────────────

void IRAM_ATTR TR_GNSSReceiverMosaicSerial::ppsIsr(void* arg)
{
    auto* self = static_cast<TR_GNSSReceiverMosaicSerial*>(arg);
    const int64_t t = esp_timer_get_time();
    const int64_t gap = t - self->pps_last_isr_us_;
    self->pps_last_isr_us_ = t;
    if (gap < kPpsMinSpacingUs)
    {
        self->pps_glitches_ = self->pps_glitches_ + 1;
        self->pps_burst_ = self->pps_burst_ + 1;
        if (self->pps_burst_ >= kPpsBurstMute)
        {
            gpio_intr_disable(static_cast<gpio_num_t>(self->pps_pin_));
            self->pps_muted_ = true;
        }
        return;
    }
    self->pps_burst_ = 0;
    const uint32_t h = self->pps_head_.load(std::memory_order_relaxed);
    self->pps_edges_[h % kPpsRing] = t;
    self->pps_head_.store(h + 1, std::memory_order_release);
}

void TR_GNSSReceiverMosaicSerial::setupPps()
{
    if (pps_pin_ < 0) return;
    // An input only: J1 pin 5 is the receiver's output on this harness. The
    // pull-down holds it low with no receiver on the connector; the
    // receiver's push-pull PPS overrides it through the harness's 1 kOhm.
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << static_cast<uint32_t>(pps_pin_);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io);
    (void)gpio_install_isr_service(0);   // ESP_ERR_INVALID_STATE: the collector's, already there
    gpio_isr_handler_add(static_cast<gpio_num_t>(pps_pin_), &ppsIsr, this);
    gpio_intr_enable(static_cast<gpio_num_t>(pps_pin_));
}

void TR_GNSSReceiverMosaicSerial::pairPps(double gps_s, int64_t arrival_us)
{
    // Take the edges the interrupt has stamped since last time.
    const uint32_t head = pps_head_.load(std::memory_order_acquire);
    if (head - pps_tail_ > kPpsRing) pps_tail_ = head - kPpsRing;   // overwritten while nobody paired
    while (pps_tail_ != head)
    {
        pairer_.addEdge(pps_edges_[pps_tail_ % kPpsRing]);
        pps_tail_++;
        pps_edges_seen_++;
    }
    int64_t edge = 0;
    if (!pairer_.pair(gps_s, arrival_us, edge)) return;

    // An edge IS its GPS second (latency 0), so the line needs no receiver
    // delay calibrated, unlike the LC86G's sentence arrivals.
    time_sync_.addSample(gps_s, edge);
    if (clock_q_ != nullptr && time_sync_.lineSeq() != clock_seq_seen_)
    {
        clock_seq_seen_ = time_sync_.lineSeq();
        lc86::GnssTimeSync::Snapshot snap;
        time_sync_.snapshot(snap);
        GnssClockData rec;
        lc86::gnssClockRecord(snap, static_cast<uint32_t>(edge), rec);
        rec.flags |= GCL_PPS;
        if (xQueueSend(clock_q_, &rec, 0) != pdTRUE) clock_dropped_++;
    }
}

void TR_GNSSReceiverMosaicSerial::noteDelay(uint16_t number, uint32_t tow_ms, uint16_t wnc, int64_t end_us)
{
    // The receiver's own delay: from its epoch (receiver time, within the
    // 0.5 ms clock-jump threshold of GPS) to the block's last byte here.
    if (tow_ms == mosaic::kTowDnu || wnc == mosaic::kWncDnu || !time_sync_.valid()) return;
    int64_t mcu = 0;
    if (!time_sync_.mcuAt(wnc * 604800.0 + tow_ms * 1.0e-3, mcu)) return;
    const double ms = static_cast<double>(end_us - mcu) * 1.0e-3;
    DelayStats& s = (number == mosaic::SBF_MEAS_EPOCH) ? delay_meas_ : delay_pvt_;
    s.n++;
    s.sum_ms += ms;
    s.max_ms = std::max(s.max_ms, ms);
}

// ── Blocks, lines, prompts (the parser's sink) ──────────────────────────

void TR_GNSSReceiverMosaicSerial::onBlock(const uint8_t* block, size_t len, size_t index)
{
    const int64_t end_us = rd_us_ - static_cast<int64_t>(
        static_cast<double>((rd_n_ - 1 - index) + rd_behind_) * byte_us_);
    link_.onBlock(nowMs());
    n_blocks_++;

    // The flight log's copy: every block, verbatim. Never blocks; a full
    // ring means nobody is draining it. Before the flight loop runs that is
    // expected (the boot), so only drops after it starts are counted, as the
    // collector counts its IMU queue's.
    if (len > GNSS_SBF_MAX_FRAME)
    {
        too_long_++;
    }
    else if (ring_ != nullptr)
    {
        const uint32_t t = static_cast<uint32_t>(end_us);
        std::memcpy(item_, &t, 4);
        std::memcpy(item_ + 4, block, len);
        if (xRingbufferSend(ring_, item_, 4 + len, 0) != pdTRUE &&
            draining_.load(std::memory_order_relaxed))
        {
            ring_dropped_++;
        }
    }

    mosaic::SbfHeader h;
    if (!mosaic::sbfHeader(block, len, h)) return;
    switch (h.number)
    {
        case mosaic::SBF_PVT_GEODETIC:
        {
            mosaic::PvtGeodetic p;
            if (!mosaic::decodePvtGeodetic(block, len, p)) break;
            mosaic::gnssDataFromPvt(p, leap_s_, pdop_x100_, data_);
            // MCU sample time, as the other drivers stamp it; the freshness
            // gates downstream key off it.
            data_.time_us = static_cast<uint32_t>(esp_timer_get_time());
            new_pvt_ = true;
            n_pvt_++;
            if (p.has_solution)
            {
                n_fix_++;
                if (p.undulation_m > -1.0e10f)
                {
                    geoid_sep_m_.store(p.undulation_m, std::memory_order_relaxed);
                    geoid_sep_valid_.store(true, std::memory_order_relaxed);
                }
            }
            noteDelay(h.number, h.tow_ms, h.wnc, end_us);
            if (h.tow_ms != mosaic::kTowDnu && h.wnc != mosaic::kWncDnu && (h.tow_ms % 1000U) == 0)
            {
                pairPps(h.wnc * 604800.0 + h.tow_ms / 1000U, end_us);
            }
            break;
        }
        case mosaic::SBF_MEAS_EPOCH:
            n_meas_++;
            noteDelay(h.number, h.tow_ms, h.wnc, end_us);
            break;
        case mosaic::SBF_MEAS_EXTRA:
            n_extra_++;
            break;
        case mosaic::SBF_DOP:
        {
            mosaic::Dop d;
            if (mosaic::decodeDop(block, len, d)) pdop_x100_ = d.pdop_x100;
            break;
        }
        case mosaic::SBF_RECEIVER_TIME:
        {
            mosaic::ReceiverTime t;
            if (mosaic::decodeReceiverTime(block, len, t) && t.delta_ls != -128) leap_s_ = t.delta_ls;
            break;
        }
        case mosaic::SBF_RECEIVER_STATUS:
            have_rx_status_ = mosaic::decodeReceiverStatus(block, len, rx_status_) || have_rx_status_;
            break;
        case mosaic::SBF_DISK_STATUS:
            (void)mosaic::decodeDiskStatus(block, len, disk_);
            break;
        case mosaic::SBF_XPPS_OFFSET:
        {
            mosaic::XppsOffset x;
            if (mosaic::decodeXppsOffset(block, len, x) && x.time_scale == 1 &&
                x.tow_ms != mosaic::kTowDnu && x.wnc != mosaic::kWncDnu)
            {
                pairPps(x.wnc * 604800.0 + (x.tow_ms + 500U) / 1000U, end_us);
            }
            break;
        }
        default:
            break;
    }
}

void TR_GNSSReceiverMosaicSerial::onLine(const char* line, size_t index)
{
    (void)index;
    if (std::strncmp(line, "$R?", 3) == 0) ESP_LOGW(TAG, "mosaic-G5 refused: %s", line);
    link_.onLine(line, nowMs());
}

void TR_GNSSReceiverMosaicSerial::onPrompt(const char* port, size_t index)
{
    (void)index;
    link_.onPrompt(port, nowMs());
}

// ── Poll path ───────────────────────────────────────────────────────────

bool TR_GNSSReceiverMosaicSerial::pollNewPVT(GNSSData &gnss_data)
{
    size_t drained = 0;
    while (drained < kMaxBytesPerPoll)
    {
        const size_t n = pump(0, kMaxBytesPerPoll - drained);
        if (n == 0) break;
        drained += n;
    }
    drainUartEvents();

    if (nav_resend_.load(std::memory_order_relaxed) && link_.running() && link_.queueCommand(kCmdNavResend))
    {
        nav_resend_.store(false, std::memory_order_relaxed);
        nav_resends_++;
    }
    runLink();

    // A muted PPS interrupt gets another chance after a quiet spell.
    if (pps_muted_)
    {
        const int64_t now = nowUs();
        if (pps_muted_at_us_ == 0) pps_muted_at_us_ = now;
        if (now - pps_muted_at_us_ > kPpsUnmuteUs)
        {
            pps_muted_at_us_ = 0;
            pps_burst_ = 0;
            pps_muted_ = false;
            gpio_intr_enable(static_cast<gpio_num_t>(pps_pin_));
        }
    }

    logStatus();

    if (!new_pvt_) return false;   // `gnss_data` untouched (u-blox contract)
    new_pvt_ = false;
    getGNSSData(gnss_data);
    return true;
}

void TR_GNSSReceiverMosaicSerial::getGNSSData(GNSSData &gnss_data)
{
    gnss_data = data_;
}

bool TR_GNSSReceiverMosaicSerial::pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us)
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
    if (!fits) short_buffer_++;   // the caller's buffer is smaller than GNSS_SBF_MAX_FRAME
    return fits;
}

bool TR_GNSSReceiverMosaicSerial::takeClockLine(GnssClockData& out)
{
    return clock_q_ != nullptr && xQueueReceive(clock_q_, &out, 0) == pdTRUE;
}

void TR_GNSSReceiverMosaicSerial::logStatus()
{
    // Every 30 s on the boot console: what a bench run needs to see.
    const int64_t now = nowUs();
    if (status_us_ == 0)
    {
        status_us_ = now;
        return;
    }
    const double span_s = static_cast<double>(now - status_us_) * 1.0e-6;
    if (span_s < 30.0) return;
    status_us_ = now;

    ESP_LOGI(TAG, "mosaic: %.1f MeasEpoch/s, %.1f MeasExtra/s, %.1f PVT/s (%.1f with a fix); "
                  "%lu blocks, CRC fails %lu, bad lengths %lu; log ring drops %lu, too long %lu; "
                  "UART overflows %lu; relinks %lu; nav re-sends %lu; poll task stack %lu B free",
             n_meas_ / span_s, n_extra_ / span_s, n_pvt_ / span_s, n_fix_ / span_s,
             static_cast<unsigned long>(n_blocks_), static_cast<unsigned long>(parser_.crcFails()),
             static_cast<unsigned long>(parser_.badLengths()), static_cast<unsigned long>(ring_dropped_),
             static_cast<unsigned long>(too_long_), static_cast<unsigned long>(fifo_overflows_),
             static_cast<unsigned long>(link_.relinks()), static_cast<unsigned long>(nav_resends_),
             static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
    if (pps_pin_ >= 0)
    {
        ESP_LOGI(TAG, "mosaic PPS: %lu edges, %lu paired, %lu glitches%s; %s",
                 static_cast<unsigned long>(pps_edges_seen_), static_cast<unsigned long>(pairer_.pairs()),
                 static_cast<unsigned long>(pps_glitches_), pps_muted_ ? ", MUTED" : "",
                 time_sync_.valid() ? "line valid" : "line not valid yet");
        if (time_sync_.valid())
        {
            ESP_LOGI(TAG, "mosaic PPS line: MCU crystal %+.2f ppm against GPS, rms %.3f ms; receiver delay "
                          "to the block's last byte: MeasEpoch mean %.1f ms (max %.1f, n %lu), PVTGeodetic "
                          "mean %.1f ms (max %.1f, n %lu)",
                     time_sync_.driftPpm(), time_sync_.envelopeRmsMs(),
                     delay_meas_.n ? delay_meas_.sum_ms / delay_meas_.n : 0.0, delay_meas_.max_ms,
                     static_cast<unsigned long>(delay_meas_.n),
                     delay_pvt_.n ? delay_pvt_.sum_ms / delay_pvt_.n : 0.0, delay_pvt_.max_ms,
                     static_cast<unsigned long>(delay_pvt_.n));
        }
    }
    if (have_rx_status_)
    {
        ESP_LOGI(TAG, "mosaic receiver: CPU %u %%, %d C, rx_error 0x%lx, ext_error 0x%x, up %lu s; "
                      "microSD %s%s%s, %.0f of %lu MB, error %u",
                 rx_status_.cpu_load, rx_status_.temp_c, static_cast<unsigned long>(rx_status_.rx_error),
                 rx_status_.ext_error, static_cast<unsigned long>(rx_status_.uptime_s),
                 !disk_.present ? "not reported" : (disk_.status & 0x01) ? "mounted" : "NOT mounted",
                 (disk_.status & 0x08) ? ", logging" : "", (disk_.status & 0x02) ? ", FULL" : "",
                 disk_.used_bytes / 1.0e6, static_cast<unsigned long>(disk_.size_mb), disk_.error);
    }
    n_meas_ = n_extra_ = n_pvt_ = n_fix_ = 0;
    delay_meas_ = DelayStats{};
    delay_pvt_ = DelayStats{};
}
