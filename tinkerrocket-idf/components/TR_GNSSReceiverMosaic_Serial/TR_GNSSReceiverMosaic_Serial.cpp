// TR_GNSSReceiverMosaic_Serial.cpp — Septentrio mosaic-G5 over ESP-IDF UART. See the header.
//
// References: mosaic-G5 Reference Guide, firmware 1.0.1 [RG]; the receiver
// itself (mosaic-G5 P3, firmware 1.0.0, permission file "Maximum Data Rate:
// Permitted up to 20 Hz"), as the 2026-10-15 spin-rig sessions logged it.
#include "TR_GNSSReceiverMosaic_Serial.h"
#if defined(TR_BENCH_MOSAIC_CN0) && TR_BENCH_MOSAIC_CN0
#include "SbfMeas.h"
#endif

#include <GnssClockRecord.h>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/idf_additions.h>   // xQueueCreateWithCaps
#include <freertos/task.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

static const char* TAG = "GNSS";

#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
// The navigation blocks by the numbers this driver streams and SbfNav decodes.
static_assert(mosaic::SBF_GPS_NAV == mosaic::kSbfGpsNav && mosaic::SBF_GAL_NAV == mosaic::kSbfGalNav &&
                  mosaic::SBF_BDS_NAV == mosaic::kSbfBdsNav && mosaic::SBF_GPS_ION == mosaic::kSbfGpsIon,
              "SbfParser.h and SbfNav.h must name the same navigation blocks");
#endif

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

#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
// The raw outputs (enableRawOutputs). Three epochs (~2.3 kB each, internal
// RAM): the GNSS task takes them every 5 ms, and two more ride out a pass it
// spends on an esoc's navigation blocks. 64 navigation blocks in PSRAM: an
// esoc sends about 35 at once, F/NAV and GEO already left out; 24 where there
// is no PSRAM. An epoch is tagged with the RxClkBias of a PVT at most
// kPvtCarryMs before it, carried by its drift and the clock jumps since (the
// PVT before each MeasEpoch is 50 ms old at 20 Hz; the rest of the two
// minutes rides out a lost sky, on the receiver's free-running clock), and
// only while some 1PPS edge has paired with its second within
// kPpsPairMaxAgeUs: the line drifts with the MCU crystal once the edges stop.
// MosaicTimeTag.h holds the rest of the rule.
static constexpr UBaseType_t kRawEpochQDepth   = 3;
static constexpr UBaseType_t kNavQDepthPsram   = 64;
static constexpr UBaseType_t kNavQDepthInternal = 24;
static constexpr int64_t     kPvtCarryMs       = 120000;
static constexpr int64_t     kPpsPairMaxAgeUs  = 120000000;
#endif

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
    : _uartPort(uart_port), begin_timeout_ms_(kBeginTimeoutMs)
{
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
    // The 1PPS line follows a step of its own edges (GnssTimeSync.h): the
    // receiver's 1PPS moves with the sky it tracks, which on the RF rig a
    // transmitter underrun delays by milliseconds. The auto image keeps the
    // line as it was.
    lc86::GnssTimeSync::Config ts;
    ts.exact_step_s = kPpsOnLineUs * 1.0e-6;
    time_sync_.setConfig(ts);
#endif
}

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
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
        // The raw-GNSS filter needs every ephemeris the receiver holds, and
        // OnChange sent them when it first decoded them, before this boot or
        // this link: ask again after each setup.
        if (raw_ != nullptr) nav_resend_.store(true, std::memory_order_relaxed);
#endif
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

    if (!begin_wait_)
    {
        // setBeginWait(false): the link's first step here (its UART at the
        // first baud it listens on), every step after it in the poll task
        runLink();
        ESP_LOGI(TAG, "Looking for the mosaic-G5 (RX=%u TX=%u PPS=%d) from the poll task, without waiting",
                 rx_pin_, tx_pin_, pps_pin_);
        return true;
    }

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
#if defined(TR_BENCH_MOSAIC_COLD_START) && TR_BENCH_MOSAIC_COLD_START
    // Bench only (RF rig): forget the real sky's time, position, ephemerides and
    // almanac before a simulated one, as the hotshot runs did over the receiver's
    // USB. The soft reset keeps the boot configuration the setup just saved.
    ESP_LOGW(TAG, "BENCH: mosaic-G5 cold start (erst,Soft,PVTData+SatData) %s",
             link_.queueCommand("erst,Soft,PVTData+SatData") ? "queued" : "NOT queued");
#endif
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
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
    // A link backed up a second (a relink at a low baud rate, the 20 Hz
    // streams queued) would hand this block the next second's edge: a line
    // labelled a second off that the PPS check cannot see (its tags and its
    // edges carry the same label). The last MeasEpoch's lateness against the
    // earliest arrival lately says how far behind the link is (MosaicJumpState).
    if (raw_ != nullptr && raw_->jump.arr_have && raw_->jump.late_us > kPairMaxLateUs)
    {
        pairs_refused_late_++;
        return;
    }
#endif
    int64_t edge = 0;
    if (!pairer_.pair(gps_s, arrival_us, edge)) return;

    // An edge IS its GPS second (latency 0), so the line needs no receiver
    // delay calibrated, unlike the LC86G's sentence arrivals.
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
    // the line steps later onto an edge only where the PVTs moved by as much:
    // a late edge with no reason is an interrupt held off (MosaicTimeTag.h)
    const bool later_ok = raw_ == nullptr || raw_->tag.edgeOnChain(std::llround(gps_s), edge);
    time_sync_.addSample(gps_s, edge, later_ok);
    if (raw_ != nullptr)
    {
        // the edge against the line it moved, which places it from the
        // seconds before, and against the chain: an edge off either holds the
        // tags back (MosaicTimeTag.h)
        int64_t at = 0;
        const bool on = time_sync_.mcuAt(gps_s, at);
        raw_->tag.notePair(std::llround(gps_s), edge, on, at, time_sync_.driftPpm() * 1.0e-6, time_sync_.restarts());
        raw_->st.tag = raw_->tag.stats();
        raw_->st.line_steps = time_sync_.steps();
        raw_->st.line_last_step_ms = time_sync_.lastStepMs();
        raw_->st.line_later_refused = time_sync_.laterRefused();
        raw_->st.line_restarts = time_sync_.restarts();
        raw_->st.pairs_refused_late = pairs_refused_late_;
    }
#else
    time_sync_.addSample(gps_s, edge);
#endif
    // The line each time it moves; on the mosaic-only image each edge paired,
    // moved or not, so a replay judges every edge against the line as the tag
    // did (an edge the line sets aside as the start of a step moves nothing)
    bool log_line = time_sync_.lineSeq() != clock_seq_seen_;
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
    log_line = true;
#endif
    if (clock_q_ != nullptr && log_line)
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

#if defined(TR_BENCH_MOSAIC_CN0) && TR_BENCH_MOSAIC_CN0
// Bench only (RF rig): every 5 s, what the receiver tracks, per system, from one
// MeasEpoch -- the L1-band signal the raw filter uses (GPS L1CA 0, Galileo E1 17,
// BeiDou B1I 28) with its C/N0 median and max, and every signal counted. Tells a
// rig run's level and acquisition apart from the fix, which the status line has.
static void benchCn0Line(const uint8_t* block, size_t len)
{
    static int64_t last_us = 0;
    const int64_t now = esp_timer_get_time();
    if (now - last_us < 5000000) return;
    last_us = now;
    mosaic::SbfMeasReader r;
    const mosaic::SbfStatus st = r.begin(block, len);
    if (st != mosaic::SbfStatus::OK)
    {
        ESP_LOGI("GNSS", "[RIG] MeasEpoch not readable yet (status %u: 5 = no time)", static_cast<unsigned>(st));
        return;
    }
    float cn[3][64];
    int n[3] = {0, 0, 0}, all = 0, other = 0;
    mosaic::SbfMeasSignal g;
    while (r.next(g))
    {
        all++;
        int k = -1;
        if (g.svid >= 1 && g.svid <= 37 && g.sig == 0) k = 0;
        else if (g.svid >= 71 && g.svid <= 106 && g.sig == 17) k = 1;
        else if (((g.svid >= 141 && g.svid <= 180) || (g.svid >= 223 && g.svid <= 245)) && g.sig == 28) k = 2;
        if (k < 0) { other++; continue; }
        if (g.cn0Valid() && n[k] < 64) cn[k][n[k]++] = static_cast<float>(g.cn0Dbhz());
    }
    float med[3] = {0, 0, 0}, mx[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++)
    {
        if (!n[k]) continue;
        std::sort(cn[k], cn[k] + n[k]);
        med[k] = cn[k][n[k] / 2];
        mx[k] = cn[k][n[k] - 1];
    }
    ESP_LOGI("GNSS", "[RIG] tracked: GPS L1CA %d (C/N0 med %.1f max %.1f) | GAL E1 %d (%.1f / %.1f) | "
                     "BDS B1I %d (%.1f / %.1f) | %d signals, %d other",
             n[0], med[0], mx[0], n[1], med[1], mx[1], n[2], med[2], mx[2], all, other);
}
#endif

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
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
            if (raw_ != nullptr) rawPvt(p);   // after the pair: its second's edge is known
#endif
            break;
        }
        case mosaic::SBF_MEAS_EPOCH:
            n_meas_++;
            noteDelay(h.number, h.tow_ms, h.wnc, end_us);
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
            if (raw_ != nullptr) rawMeasEpoch(block, len, h.tow_ms, end_us);
#endif
#if defined(TR_BENCH_MOSAIC_CN0) && TR_BENCH_MOSAIC_CN0
            benchCn0Line(block, len);
#endif
            break;
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
        case mosaic::SBF_GPS_NAV:
        case mosaic::SBF_GAL_NAV:
        case mosaic::SBF_BDS_NAV:
            if (raw_ != nullptr) rawNav(block, len, h.number);
            break;
        case mosaic::SBF_GPS_ION:
            if (raw_ != nullptr) rawIon(block, len);
            break;
#endif
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
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
            if (raw_ != nullptr && mosaic::decodeReceiverTime(block, len, t))
                raw_->tag.noteFineTime((t.sync_level & 0x04) != 0);   // FINETIME: the time tag's precondition
#endif
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
#if defined(TR_BENCH_MOSAIC_TAG_TRACE) && TR_BENCH_MOSAIC_TAG_TRACE
                // the pulse's SyncAge: how long since the receiver last aligned it to its fix
                xpps_s_ = static_cast<int64_t>(x.wnc) * 604800 + (x.tow_ms + 500U) / 1000U;
                xpps_sync_age_ = x.sync_age_s;
#endif
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
        ESP_LOGI(TAG, "mosaic PPS: %lu edges, %lu paired, %lu glitches%s; %s; clock records dropped %lu",
                 static_cast<unsigned long>(pps_edges_seen_), static_cast<unsigned long>(pairer_.pairs()),
                 static_cast<unsigned long>(pps_glitches_), pps_muted_ ? ", MUTED" : "",
                 time_sync_.valid() ? "line valid" : "line not valid yet", static_cast<unsigned long>(clock_dropped_));
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
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
    if (raw_ != nullptr) logRawStatus();
#endif
    n_meas_ = n_extra_ = n_pvt_ = n_fix_ = 0;
    delay_meas_ = DelayStats{};
    delay_pvt_ = DelayStats{};
}

// ── Raw outputs for the raw-GNSS navigation filter ──────────────────────
// The mosaic-only image's alone (TR_GNSS_DRIVER_MOSAIC; the header says why).
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC

bool TR_GNSSReceiverMosaicSerial::enableRawOutputs()
{
    if (raw_ != nullptr) return true;
    // Internal RAM: the poll task adapts each epoch into it at 10 Hz. Only the
    // navigation queue goes to PSRAM, where its 10 kB costs the flight nothing.
    void* mem = heap_caps_malloc(sizeof(RawState), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (mem == nullptr)
    {
        ESP_LOGE(TAG, "mosaic raw: no memory for the raw outputs (%u B)", static_cast<unsigned>(sizeof(RawState)));
        return false;
    }
    RawState* r = new (mem) RawState();
    r->epoch_q = xQueueCreate(kRawEpochQDepth, sizeof(gnss_raw::TimedRawEpoch));
    r->nav_q = xQueueCreateWithCaps(kNavQDepthPsram, sizeof(NavItem), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (r->nav_q != nullptr)
    {
        r->nav_caps = true;
        r->st.nav_in_psram = true;
        r->st.nav_depth = static_cast<uint16_t>(kNavQDepthPsram);
    }
    else
    {
        r->nav_q = xQueueCreate(kNavQDepthInternal, sizeof(NavItem));
        r->st.nav_depth = static_cast<uint16_t>(kNavQDepthInternal);
    }
    if (r->epoch_q == nullptr || r->nav_q == nullptr)
    {
        // nothing half made: give back what was got
        ESP_LOGE(TAG, "mosaic raw: no memory for the epoch queue (%u B) or the navigation queue",
                 static_cast<unsigned>(kRawEpochQDepth * sizeof(gnss_raw::TimedRawEpoch)));
        if (r->epoch_q != nullptr) vQueueDelete(r->epoch_q);
        if (r->nav_q != nullptr)
        {
            if (r->nav_caps) vQueueDeleteWithCaps(r->nav_q);
            else vQueueDelete(r->nav_q);
        }
        r->~RawState();
        heap_caps_free(mem);
        return false;
    }
    mosaic::MosaicTimeTag::Config tc;
    tc.carry_ms = kPvtCarryMs;
    tc.pair_max_age_us = kPpsPairMaxAgeUs;
    tc.bar_us = kPpsTagBarUs;
    tc.on_line_us = kPpsOnLineUs;
    r->tag = mosaic::MosaicTimeTag(tc);
    r->st.enabled = true;
    raw_ = r;   // before begin(): the poll task does not exist yet
    ESP_LOGI(TAG, "mosaic raw: outputs on -- %u epochs (%u B, internal), %u navigation blocks in %s, %u B of state",
             static_cast<unsigned>(kRawEpochQDepth),
             static_cast<unsigned>(kRawEpochQDepth * sizeof(gnss_raw::TimedRawEpoch)),
             static_cast<unsigned>(r->st.nav_depth), r->st.nav_in_psram ? "PSRAM" : "internal RAM",
             static_cast<unsigned>(sizeof(RawState)));
    return true;
}

void TR_GNSSReceiverMosaicSerial::setRawEpochPeriodMs(uint32_t period_ms)
{
    if (raw_ == nullptr) return;
    raw_->period_ms.store(period_ms, std::memory_order_relaxed);
    raw_->st.period_ms = period_ms;
}

bool TR_GNSSReceiverMosaicSerial::setRawDopplerPairing(const mosaic::MosaicDopplerPairConfig& c)
{
    if (raw_ == nullptr) return false;
    if (raw_->pairing.load(std::memory_order_acquire) != nullptr) return true;
    // Internal RAM, as the rest of the raw state: the poll task reads it at
    // 20 Hz. Filled before it is published, so the poll task never sees it half made
    void* mem = heap_caps_malloc(sizeof(RawState::Pairing), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (mem == nullptr)
    {
        ESP_LOGE(TAG, "mosaic raw: no memory for the Doppler pairs (%u B): every epoch's own Dopplers",
                 static_cast<unsigned>(sizeof(RawState::Pairing)));
        return false;
    }
    RawState::Pairing* p = new (mem) RawState::Pairing();
    p->cfg = c;
    raw_->st.pair_on = true;
    raw_->pairing.store(p, std::memory_order_release);
    ESP_LOGI(TAG, "mosaic raw: Dopplers in pairs -- each grid epoch's with the epoch %lu ms before it, at most "
                  "%.1f m/s apart, C/N0 floors %.1f/%.1f/%.1f dB-Hz (GPS/GAL/BDS), %u B of state",
             static_cast<unsigned long>(c.dt_ms), c.max_diff_mps, static_cast<double>(c.cn0_min[GNSS_SYS_GPS]),
             static_cast<double>(c.cn0_min[GNSS_SYS_GAL]), static_cast<double>(c.cn0_min[GNSS_SYS_BDS]),
             static_cast<unsigned>(sizeof(RawState::Pairing)));
    return true;
}

bool TR_GNSSReceiverMosaicSerial::takeRawEpoch(gnss_raw::TimedRawEpoch& out)
{
    if (raw_ == nullptr) return false;
    raw_->taking_epochs.store(true, std::memory_order_relaxed);
    return xQueueReceive(raw_->epoch_q, &out, 0) == pdTRUE;
}

bool TR_GNSSReceiverMosaicSerial::takeEphemeris(uint8_t* out, size_t cap, size_t& len)
{
    if (raw_ == nullptr) return false;
    raw_->taking_nav.store(true, std::memory_order_relaxed);
    NavItem it;
    while (xQueueReceive(raw_->nav_q, &it, 0) == pdTRUE)
    {
        if (it.len > cap)
        {
            raw_->st.nav_short_buffer++;   // this task's own counter
            continue;
        }
        std::memcpy(out, it.bytes, it.len);
        len = it.len;
        return true;
    }
    return false;
}

bool TR_GNSSReceiverMosaicSerial::broadcastIono(int8_t raw[8], uint16_t& gps_day) const
{
    if (raw_ == nullptr) return false;
    portENTER_CRITICAL(&raw_mux_);
    const bool have = raw_->iono_have;
    if (have)
    {
        std::memcpy(raw, raw_->iono_raw, sizeof(raw_->iono_raw));
        gps_day = raw_->iono_day;
    }
    portEXIT_CRITICAL(&raw_mux_);
    return have;
}

void TR_GNSSReceiverMosaicSerial::rawStats(RawStats& out) const
{
    out = (raw_ != nullptr) ? raw_->st : RawStats{};
}

void TR_GNSSReceiverMosaicSerial::rawMeasEpoch(const uint8_t* block, size_t len, uint32_t tow_ms, int64_t end_us)
{
    RawState& r = *raw_;
    mosaic::MosaicEpochInfo info;
    RawState::Pairing* const pairing = r.pairing.load(std::memory_order_acquire);
    if (!mosaic::onFuseGrid(tow_ms, r.period_ms.load(std::memory_order_relaxed)))
    {
        // Off the grid: its header's clock jumps only, so the bias carried to
        // the next epoch on it misses none (MosaicRawEpoch.h); with the
        // Dopplers in pairs, its Dopplers too, for the next grid epoch
        if (pairing == nullptr)
        {
            (void)mosaic::noteMeasEpochClock(block, len, r.jump, info, end_us);
        }
        else
        {
            const int64_t t0 = nowUs();
            (void)mosaic::noteMeasEpochDopplers(block, len, r.jump, info, pairing->prev, end_us);
            const uint32_t us = static_cast<uint32_t>(nowUs() - t0);
            r.st.pair_reads++;
            r.st.pair_read_us_sum += us;
            if (us > r.st.pair_read_us_max) r.st.pair_read_us_max = us;
        }
        r.st.decimated++;
        r.st.clock_jumps = r.jump.jumps;
        r.st.clock_restarts = r.jump.rebases;
        r.st.clock_resets = r.jump.resets;
        r.st.clock_resolved = r.jump.resolved;
        return;
    }
    gnss_raw::TimedRawEpoch& e = r.scratch;
    // the arrival places a jump the count shows only mod 256 (MosaicJumpState)
    const int64_t t_adapt = pairing != nullptr ? nowUs() : 0;
    const bool cells = mosaic::rawEpochFromMeasEpoch(block, len, e.epoch, r.jump, info, end_us);
    r.st.clock_jumps = r.jump.jumps;
    r.st.clock_restarts = r.jump.rebases;
    r.st.clock_resets = r.jump.resets;
    r.st.clock_resolved = r.jump.resolved;
    if (pairing != nullptr)
    {
        const int64_t t_avg = nowUs();
        const uint32_t us = static_cast<uint32_t>(t_avg - t_adapt);
        r.st.pair_adapts++;
        r.st.pair_adapt_us_sum += us;
        if (us > r.st.pair_adapt_us_max) r.st.pair_adapt_us_max = us;
        // each Doppler with the epoch before's, where the two are a pair
        // (MosaicRawEpoch.h): the epoch is handed over with the means
        if (cells && info.status == mosaic::SbfStatus::OK)
        {
            const int64_t rx = static_cast<int64_t>(info.wnc) * 604800000LL + info.tow_ms;
            (void)mosaic::averageDopplerPairs(e.epoch, rx, info.jumped || info.rebased, pairing->prev,
                                              pairing->cfg, r.st.pair);
            const uint32_t avg_us = static_cast<uint32_t>(nowUs() - t_avg);
            if (avg_us > r.st.pair_avg_us_max) r.st.pair_avg_us_max = avg_us;
        }
        pairing->prev.valid = false;   // one grid epoch's mate only
    }
    if (info.status != mosaic::SbfStatus::OK)
    {
        r.st.empty++;   // refused: no time to place
        return;
    }

    // t_gps = TOW - RxClkBias: the bias of the last PVT that anchors the jump
    // count, run on by its drift and by the clock jumps since (dtrMs), on the
    // PPS line; handed over timed only where MosaicTimeTag says it is right
    const int64_t rx_ms = static_cast<int64_t>(info.wnc) * 604800000LL + info.tow_ms;
    double dtr = 0.0;
    const bool carried = r.tag.carry(info, r.jump, dtr);
    int64_t mcu = 0;
    const bool placed = carried && time_sync_.mcuAt(mosaic::epochGpsS(info.wnc, info.tow_ms, dtr), mcu);
    const mosaic::MosaicTimeTag::Why why = r.tag.judge(time_sync_.valid(), carried, nowUs());
    const bool timed = placed && why == mosaic::MosaicTimeTag::Why::TIMED;
    const bool whole = info.tow_ms % 1000U == 0;
    if (whole)
    {
        r.sec_rx_ms = rx_ms;
        r.sec_dtr_ms = dtr;
        r.sec_placed = placed;
        r.sec_why = why;
    }

    bool handed = false;
    if (!cells)
    {
        r.st.empty++;   // nothing the filter takes: not handed over (an empty epoch is no
                        // epoch to the levelling's steady test either)
    }
    else
    {
        e.arrival_us = end_us;
        e.gps_week = info.wnc;
        e.mcu_us = timed ? mcu : 0;
        // what the tag took off the time of week, with the tag only: the rig
        // replay's clock takes the timed ones' GPS instant (TOW - dtr), and
        // an untimed or held epoch carries none to take
        e.tag_dtr_ms = timed ? static_cast<float>(dtr) : 0.0f;
        if (xQueueSend(r.epoch_q, &e, 0) == pdTRUE)
        {
            handed = true;
            r.st.published++;
            r.tag.countEpoch(timed ? mosaic::MosaicTimeTag::Why::TIMED : why);
            if (!timed) r.st.untimed++;
        }
        else if (r.taking_epochs.load(std::memory_order_relaxed))
        {
            r.st.dropped++;   // full: the GNSS task is not taking them (yet, at boot: not counted)
        }
    }
    // a whole second's tag, handed over timed or held back, for the PPS check
    // when its own PVT comes: the held ones' checks are what release the hold
    if (whole && placed) r.tag.noteWholeSecond(rx_ms, mcu, handed && timed);
    r.st.tag = r.tag.stats();
}

void TR_GNSSReceiverMosaicSerial::rawPvt(const mosaic::PvtGeodetic& p)
{
    RawState& r = *raw_;
    // The PPS tag check of this second's epoch, then this PVT's bias as the
    // anchor (MosaicTimeTag.h). The check judges an edge the interrupt
    // stamped late against where the line's second puts it after the edge
    // before (notePpsTag).
    int64_t second_us = 1000000;
    if (p.tow_ms != mosaic::kTowDnu && p.wnc != mosaic::kWncDnu && p.tow_ms % 1000U == 0)
    {
        const double gs = p.wnc * 604800.0 + p.tow_ms / 1000U;
        int64_t at_s = 0, at_before = 0;
        if (time_sync_.mcuAt(gs, at_s) && time_sync_.mcuAt(gs - 1.0, at_before)) second_us = at_s - at_before;
    }
    r.tag.notePvt(p, r.jump, second_us, time_sync_.driftFitted());
    r.st.tag = r.tag.stats();
#if defined(TR_BENCH_MOSAIC_TAG_TRACE) && TR_BENCH_MOSAIC_TAG_TRACE
    if (p.tow_ms != mosaic::kTowDnu && p.wnc != mosaic::kWncDnu && p.tow_ms % 1000U == 0) benchTagTrace(p);
#endif
}

void TR_GNSSReceiverMosaicSerial::rawNav(const uint8_t* block, size_t len, uint16_t number)
{
    RawState& r = *raw_;
    const int k = (number == mosaic::SBF_GPS_NAV) ? 0 : (number == mosaic::SBF_GAL_NAV) ? 1 : 2;
    if (len > mosaic::kSbfNavMaxLen)
    {
        r.st.nav_too_long++;
        return;
    }
    // What the decoder would refuse for its source or orbit (Galileo F/NAV,
    // BeiDou GEO) or its length never takes a queue place
    if (!mosaic::sbfNavKeep(block, len))
    {
        r.st.nav_filtered[k]++;
        return;
    }
    r.nav_item.len = static_cast<uint8_t>(len);
    std::memcpy(r.nav_item.bytes, block, len);
    if (xQueueSend(r.nav_q, &r.nav_item, 0) == pdTRUE)
        r.st.nav_queued[k]++;
    else
        r.st.nav_dropped[k]++;   // a lost ephemeris until the next resend: counted from boot
}

void TR_GNSSReceiverMosaicSerial::rawIon(const uint8_t* block, size_t len)
{
    RawState& r = *raw_;
    int8_t raw[8];
    uint16_t day = 0;
    if (!mosaic::decodeGpsIon(block, len, raw, day))
    {
        r.st.ions_bad++;
        return;
    }
    portENTER_CRITICAL(&raw_mux_);
    std::memcpy(r.iono_raw, raw, sizeof(r.iono_raw));
    r.iono_day = day;
    r.iono_have = true;
    portEXIT_CRITICAL(&raw_mux_);
    r.st.ions++;
}

void TR_GNSSReceiverMosaicSerial::logRawStatus()
{
    using Why = mosaic::MosaicTimeTag::Why;
    using Break = mosaic::MosaicTimeTag::Break;
    const RawStats& s = raw_->st;
    const mosaic::MosaicTimeTag::Stats& t = s.tag;
    auto why = [&](Why w) { return static_cast<unsigned long>(t.why[static_cast<int>(w)]); };
    auto brk = [&](Break b) { return static_cast<unsigned long>(t.breaks[static_cast<int>(b)]); };
    const mosaic::MosaicTimeTag& tag = raw_->tag;
    ESP_LOGI(TAG, "mosaic raw: epochs on the %lu ms grid: %lu handed over (%lu untimed: no line %lu, not FINETIME %lu, "
                  "no edge %lu, no bias %lu, edge off the line %lu, edge off the chain %lu, held %lu), %lu dropped, "
                  "%lu empty; %lu off the grid; last whole second %s | nav queued/filtered/dropped: GPS %lu/%lu/%lu, "
                  "GAL %lu/%lu/%lu, BDS %lu/%lu/%lu; too long %lu, short buffer %lu (queue %u in %s) | GPSIon %lu "
                  "(%lu refused) | clock jumps %lu, restarts %lu (clock lost %lu; jumps placed past 256 ms %lu), "
                  "PVTs not carried %lu",
             static_cast<unsigned long>(s.period_ms), static_cast<unsigned long>(s.published),
             static_cast<unsigned long>(s.untimed), why(Why::NO_LINE), why(Why::NOT_FINETIME), why(Why::PPS_STALE),
             why(Why::NO_BIAS), why(Why::EDGE_OFF), why(Why::OFF_CHAIN), why(Why::HELD),
             static_cast<unsigned long>(s.dropped), static_cast<unsigned long>(s.empty),
             static_cast<unsigned long>(s.decimated), mosaic::MosaicTimeTag::whyName(raw_->sec_why),
             static_cast<unsigned long>(s.nav_queued[0]), static_cast<unsigned long>(s.nav_filtered[0]),
             static_cast<unsigned long>(s.nav_dropped[0]), static_cast<unsigned long>(s.nav_queued[1]),
             static_cast<unsigned long>(s.nav_filtered[1]), static_cast<unsigned long>(s.nav_dropped[1]),
             static_cast<unsigned long>(s.nav_queued[2]), static_cast<unsigned long>(s.nav_filtered[2]),
             static_cast<unsigned long>(s.nav_dropped[2]), static_cast<unsigned long>(s.nav_too_long),
             static_cast<unsigned long>(s.nav_short_buffer), static_cast<unsigned>(s.nav_depth),
             s.nav_in_psram ? "PSRAM" : "internal RAM", static_cast<unsigned long>(s.ions),
             static_cast<unsigned long>(s.ions_bad), static_cast<unsigned long>(s.clock_jumps),
             static_cast<unsigned long>(s.clock_restarts), static_cast<unsigned long>(s.clock_resets),
             static_cast<unsigned long>(s.clock_resolved), static_cast<unsigned long>(t.pvt_passed));
    ESP_LOGI(TAG, "mosaic PPS tag check: %lu whole seconds judged, |residual| max %ld us, %lu over %ld us, "
                  "last %+ld us | %lu before the line had the crystal's drift (max %ld us, not judged) | %lu edges "
                  "stamped late (the interrupt held off), up to %ld us past due: judged where due | %lu held back "
                  "(checked, not judged: max %ld us, last %+ld us)",
             static_cast<unsigned long>(t.pps.checks), static_cast<long>(t.pps.max_us),
             static_cast<unsigned long>(t.pps.over), static_cast<long>(kPpsTagBarUs),
             static_cast<long>(t.pps.last_us), static_cast<unsigned long>(t.pps.settling),
             static_cast<long>(t.pps.settling_max_us), static_cast<unsigned long>(t.pps.late),
             static_cast<long>(t.pps.late_max_us), static_cast<unsigned long>(t.pps.held),
             static_cast<long>(t.pps.held_max_us), static_cast<long>(t.pps.held_last_us));
    unsigned long holds = 0;
    for (int b = 1; b < static_cast<int>(Break::COUNT); ++b) holds += t.breaks[b];
    ESP_LOGI(TAG, "mosaic tag: %s%s; holds %lu (first anchor %lu, receiver clock lost %lu, jump count restart %lu, "
                  "anchor expired %lu, bias step %lu (last %+.3f ms), check failed %lu, line restart %lu, edge off "
                  "the chain %lu), released by the 1PPS %lu | since the reference: PVTs %+.3f ms, last edge %+.3f ms "
                  "(%s); references dropped %lu | edges off the line by over %ld us: %lu (last %+lld us), off the "
                  "chain: %lu | the line stepped onto its edges %lu times (last %+.3f ms), set aside %lu later edges "
                  "with no PVT step, restarted %lu times | pairings refused with the link behind: %lu",
             tag.held() ? "HELD after a " : "trusted", tag.held() ? mosaic::MosaicTimeTag::breakName(tag.holdBreak()) : "",
             holds, brk(Break::FIRST), brk(Break::RESET), brk(Break::REBASE), brk(Break::EXPIRED),
             brk(Break::BIAS_STEP), t.last_bias_step_ms, brk(Break::CHECK_FAILED), brk(Break::LINE_RESTART),
             brk(Break::OFF_CHAIN), static_cast<unsigned long>(t.confirmed), tag.chainMs(),
             tag.edgeRefUs() * 1.0e-3, !tag.refHave() ? "no reference" : (tag.onChain() ? "on the chain" : "OFF the chain"),
             static_cast<unsigned long>(t.refs_dropped), static_cast<long>(kPpsOnLineUs),
             static_cast<unsigned long>(t.edges_off), static_cast<long long>(tag.edgeOffLineUs()),
             static_cast<unsigned long>(t.edges_off_chain), static_cast<unsigned long>(s.line_steps),
             s.line_last_step_ms, static_cast<unsigned long>(s.line_later_refused),
             static_cast<unsigned long>(s.line_restarts), static_cast<unsigned long>(s.pairs_refused_late));
}

#if defined(TR_BENCH_MOSAIC_TAG_TRACE) && TR_BENCH_MOSAIC_TAG_TRACE
// Bench only (RF rig): every whole second, at its PVTGeodetic, what the time
// tag was made of and what the 1PPS said of it -- the receiver's time and its
// own RxClkBias, its jump count, the bias the tag carried (dtr), the edge
// paired with the second and its offset from the line, how far the PVTs and
// the edge (and the line) moved since the reference (they agree: on the
// chain), the pulse's SyncAge, the PPS check's residual, and whether the
// epoch was handed over timed. No position.
void TR_GNSSReceiverMosaicSerial::benchTagTrace(const mosaic::PvtGeodetic& p)
{
    RawState& r = *raw_;
    const mosaic::MosaicTimeTag& tag = r.tag;
    const int64_t rx_ms = static_cast<int64_t>(p.wnc) * 604800000LL + p.tow_ms;
    const bool fix = p.has_solution && std::isfinite(p.rx_clk_bias_ms) &&
                     std::fabs(p.rx_clk_bias_ms) <= mosaic::MosaicTimeTag::kMaxBiasMs;
    char dtr[48];
    if (r.sec_rx_ms != rx_ms)
        std::snprintf(dtr, sizeof(dtr), "no epoch");
    else if (!r.sec_placed)
        std::snprintf(dtr, sizeof(dtr), "not placed");
    else
        std::snprintf(dtr, sizeof(dtr), "%+.4f ms", r.sec_dtr_ms);
    const bool edge_now = tag.pairHave() && tag.pairGpsS() * 1000LL == rx_ms;
    char edge[64];
    if (edge_now)
        std::snprintf(edge, sizeof(edge), "%lld us, line %+lld us", static_cast<long long>(tag.pairEdgeUs()),
                      static_cast<long long>(tag.edgeOffLineUs()));
    else
        std::snprintf(edge, sizeof(edge), "none");
    // since the reference: the PVTs' displacement, the last edge's and the line's (the edge's less its offset
    // from the line); on the chain while the first two agree
    char chain[112];
    if (tag.refHave())
        std::snprintf(chain, sizeof(chain), "PVTs %+.3f ms, edge %+.3f ms, line %+.3f ms (%s), %lld s old",
                      tag.chainMs(), tag.edgeRefUs() * 1.0e-3, (tag.edgeRefUs() - tag.edgeOffLineUs()) * 1.0e-3,
                      tag.onChain() ? "on the chain" : "OFF the chain",
                      static_cast<long long>((tag.pairEdgeUs() - tag.refMcuUs()) / 1000000));
    else
        std::snprintf(chain, sizeof(chain), "no reference (PVTs %+.3f ms)", tag.chainMs());
    char sync[24];
    if (xpps_sync_age_ >= 0 && xpps_s_ * 1000LL == rx_ms)
        std::snprintf(sync, sizeof(sync), "%d s", xpps_sync_age_);
    else
        std::snprintf(sync, sizeof(sync), "none");
    const mosaic::MosaicTimeTag::LastCheck& c = tag.lastCheck();
    char res[64];
    if (c.have && c.rx_ms == rx_ms)
        std::snprintf(res, sizeof(res), "%+ld us (%s%s)", static_cast<long>(c.residual_us),
                      c.published ? "handed over" : "held back", c.on_chain ? "" : ", edge off the chain");
    else
        std::snprintf(res, sizeof(res), "none");
    ESP_LOGI(TAG, "[TAG] WNc %u TOW %lu | PVT %s RxClkBias %+.4f ms drift %+.3f ppm | CumClkJumps %u j %+ld ms "
                  "jsum %+lld ms restarts %lu lost %lu late %lld ms | dtr %s | edge %s, SyncAge %s | since ref: %s | "
                  "residual %s | epoch %s; %s%s, passes %u | line %s%s, steps %lu, restarts %lu",
             static_cast<unsigned>(p.wnc), static_cast<unsigned long>(p.tow_ms / 1000U), fix ? "fix" : "no fix",
             fix ? p.rx_clk_bias_ms : 0.0, fix ? static_cast<double>(p.rx_clk_drift_ppm) : 0.0,
             static_cast<unsigned>(r.jump.last_cum), static_cast<long>(r.jump.j_ms),
             static_cast<long long>(r.jump.jsum_ms), static_cast<unsigned long>(r.jump.rebases),
             static_cast<unsigned long>(r.jump.resets), static_cast<long long>(r.jump.late_us / 1000), dtr, edge,
             sync, chain, res, r.sec_rx_ms == rx_ms ? mosaic::MosaicTimeTag::whyName(r.sec_why) : "none",
             tag.held() ? "held after a " : "trusted", tag.held() ? mosaic::MosaicTimeTag::breakName(tag.holdBreak()) : "",
             static_cast<unsigned>(tag.passes()), time_sync_.valid() ? "valid" : "not valid",
             time_sync_.driftFitted() ? " fitted" : "", static_cast<unsigned long>(time_sync_.steps()),
             static_cast<unsigned long>(time_sync_.restarts()));
}
#endif
#endif  // TR_GNSS_DRIVER_MOSAIC
