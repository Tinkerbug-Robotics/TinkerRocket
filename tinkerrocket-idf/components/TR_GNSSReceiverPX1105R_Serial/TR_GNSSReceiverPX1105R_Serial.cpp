// TR_GNSSReceiverPX1105R_Serial.cpp — see TR_GNSSReceiverPX1105R_Serial.h
#include "TR_GNSSReceiverPX1105R_Serial.h"

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_memory_utils.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>

#include <GnssClockRecord.h>   // TR_GNSSReceiverLC86_Serial: GNSS_CLOCK_MSG from the line

#include <algorithm>
#include <cmath>

// GLONASS (TR_RAW_GLONASS: flight_computer/CMakeLists.txt and main/config.h, off unless set): the receiver asked
// for its strings (the 0x1E subframe mask's bit 1, LinkConfig::subframes), each slot's frequency number noted from
// the 0xE5, the strings decoded into ephemerides (SkyTraqNav.h), and the 0xE5's GLONASS cells handed over with the
// rest (Px1105rRawEpoch.h glonass). Off, the image is as before.
#ifndef TR_RAW_GLONASS
#define TR_RAW_GLONASS 0
#endif
// The L5 band (TR_RAW_L5: flight_computer/CMakeLists.txt and main/config.h, off unless set): the 0xE5's GPS L5,
// Galileo E5a and BeiDou B2a cells handed over beside the L1 band's (Px1105rRawEpoch.h l5), each L5-band code
// moved onto its satellite's L1-band one where that is gone (CodeBridge.h, its own block). Off, the image is as
// before.
#ifndef TR_RAW_L5
#define TR_RAW_L5 0
#endif
#include <cstdio>
#include <cstring>
#include <new>

static const char* TAG = "GNSS";

static_assert(skytraq::kMaxBody + 7 <= GNSS_SBF_MAX_FRAME,
              "a whole SkyTraq frame must fit the flight log's longest GNSS record");

// 921600 carries the 20 Hz set (~30 kB/s on the open sky, 2026-09-26: an
// 0xE5 of ~1.4 kB and an 0xDF each epoch) at a third of the line, and it is
// the PX1105R's fastest (0x05's codes stop there).
static constexpr uint32_t kTargetBaud = 921600U;

// As the mosaic's: the receiver powers up with the FC (the GNSS rail).
static constexpr uint32_t kBeginTimeoutMs = 30000U;

// 8 kB holds ~5 epochs; the poll task runs every 25 ms.
static constexpr size_t kUartRxBuf = 8192;
static constexpr size_t kUartTxBuf = 512;
// Interrupt at half the 128-byte FIFO: at 921600 the other half is 0.7 ms.
static constexpr int kRxFullThreshold = 64;
static constexpr size_t kMaxBytesPerPoll = 4096U;

// The raw outputs: three epochs (internal RAM), 48 ephemerides (a cold start
// completes ~25 within the first minute), 4 fixes.
static constexpr UBaseType_t kRawEpochQDepth = 3;
static constexpr UBaseType_t kEphQDepth = 48;
static constexpr UBaseType_t kFixQDepth = 4;

// GPS - UTC since 2017-01-01 (no leap second announced since): the receiver's
// binary output names no leap second, and GNSSData's clock is UTC.
static constexpr int kGpsUtcLeapS = 18;

static inline int64_t  nowUs() { return esp_timer_get_time(); }
static inline uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

namespace
{

// WGS84 ECEF to geodetic (rad, rad, m above the ellipsoid): Bowring's, a
// millimetre at the surface.
void ecefToLla(const double x[3], double& lat, double& lon, double& h)
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double b = a * (1.0 - f);
    constexpr double e2 = f * (2.0 - f);
    constexpr double ep2 = e2 / (1.0 - e2);
    const double p = std::sqrt(x[0] * x[0] + x[1] * x[1]);
    lon = std::atan2(x[1], x[0]);
    const double th = std::atan2(x[2] * a, p * b);
    const double st = std::sin(th), ct = std::cos(th);
    lat = std::atan2(x[2] + ep2 * b * st * st * st, p - e2 * a * ct * ct * ct);
    const double sl = std::sin(lat);
    const double n = a / std::sqrt(1.0 - e2 * sl * sl);
    h = (std::fabs(std::cos(lat)) > 1e-9) ? p / std::cos(lat) - n : std::fabs(x[2]) - b;
}

int32_t roundToI32(double v)
{
    if (!(v < 2147483647.0)) return 2147483647;
    if (!(v > -2147483648.0)) return -2147483647 - 1;
    return static_cast<int32_t>(std::lround(v));
}

uint8_t accuracyM(double m)
{
    if (!(m >= 0.0)) return 255;
    return m >= 255.0 ? 255 : static_cast<uint8_t>(std::lround(m));
}

// Days since 1970-01-01 to a civil date (Howard Hinnant's algorithm).
void civilFromDays(int64_t z, int& y, unsigned& m, unsigned& d)
{
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t yy = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = static_cast<int>(yy + (m <= 2 ? 1 : 0));
}

// GNSSData from one 0xDF (and the last 0xE7's satellites used). The height is
// above the ellipsoid: the PX1105R's binary output carries no geoid.
void gnssDataFromRcvState(const skytraq::RcvState& s, uint8_t used, GNSSData& out)
{
    std::memset(&out, 0, sizeof out);
    // UTC from GPS time (week 0 began 1980-01-06, day 3657 after the epoch)
    const double t = s.week * 604800.0 + s.tow_s - kGpsUtcLeapS;
    if (s.week > 0 && t > 0.0)
    {
        const int64_t whole_ms = static_cast<int64_t>(std::floor(t * 1000.0 + 0.5));
        const int64_t secs = whole_ms / 1000;
        const int64_t days = secs / 86400 + 3657;
        const int64_t sod = secs % 86400;
        int y;
        unsigned mo, d;
        civilFromDays(days, y, mo, d);
        out.year = static_cast<uint16_t>(y);
        out.month = static_cast<uint8_t>(mo);
        out.day = static_cast<uint8_t>(d);
        out.hour = static_cast<uint8_t>(sod / 3600);
        out.minute = static_cast<uint8_t>((sod / 60) % 60);
        out.second = static_cast<uint8_t>(sod % 60);
        out.milli_second = static_cast<uint16_t>(whole_ms % 1000);
    }
    const uint32_t pdop_x10 = std::isfinite(s.pdop) && s.pdop > 0.0f ? static_cast<uint32_t>(s.pdop * 10.0f + 0.5f) : 0U;
    out.pdop_x10 = static_cast<uint8_t>(pdop_x10 > 255U ? 255U : pdop_x10);
    out.num_sats = used;
    out.h_acc_m = 255;
    out.v_acc_m = 255;
    switch (s.nav_state)
    {
    case skytraq::kNavFix2d: out.fix_mode = 2; break;
    case skytraq::kNavFix3d:
    case skytraq::kNavDiff: out.fix_mode = 3; break;
    case skytraq::kNavPrediction: out.fix_mode = 1; break;
    default: out.fix_mode = 0; break;
    }
    if (out.fix_mode < 2)
        return;   // the position fields stay 0, as the other drivers leave them
    double lat, lon, h;
    ecefToLla(s.ecef_m, lat, lon, h);
    out.lat_e7 = roundToI32(lat * (180.0 / M_PI) * 1.0e7);
    out.lon_e7 = roundToI32(lon * (180.0 / M_PI) * 1.0e7);
    out.alt_mm = roundToI32(h * 1000.0);
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const double vx = s.vel_mps[0], vy = s.vel_mps[1], vz = s.vel_mps[2];
    const double ve = -so * vx + co * vy;
    const double vn = -sl * co * vx - sl * so * vy + cl * vz;
    const double vu = cl * co * vx + cl * so * vy + sl * vz;
    out.vel_e_mmps = roundToI32(ve * 1000.0);
    out.vel_n_mmps = roundToI32(vn * 1000.0);
    out.vel_u_mmps = roundToI32(vu * 1000.0);
    // No accuracy in 0xDF: the DOPs times a 3 m range error, a placeholder
    // with the receiver's own geometry in it (255 = unknown above that)
    if (std::isfinite(s.hdop) && s.hdop > 0.0f) out.h_acc_m = accuracyM(3.0 * s.hdop);
    if (std::isfinite(s.vdop) && s.vdop > 0.0f) out.v_acc_m = accuracyM(3.0 * s.vdop);
}

}  // namespace

TR_GNSSReceiverPX1105RSerial::TR_GNSSReceiverPX1105RSerial(uart_port_t uart_port)
    : _uartPort(uart_port), begin_timeout_ms_(kBeginTimeoutMs)
{
}

void TR_GNSSReceiverPX1105RSerial::setEpochLatencyS(double s)
{
    lc86::GnssTimeSync::Config ts = time_sync_.config();
    ts.latency_s = s;
    time_sync_.setConfig(ts);
}

// ── UART ────────────────────────────────────────────────────────────────

bool TR_GNSSReceiverPX1105RSerial::uartBegin(uint32_t baud)
{
    // Installed once; after that a probe's baud change only re-rates the port
    // (the mosaic driver's rule). No ESP_ERROR_CHECK: a port that will not come
    // up means GNSS-absent, never an abort in the boot.
    byte_us_ = 1.0e7 / static_cast<double>(baud);   // 8N1
    if (uart_is_driver_installed(_uartPort))
    {
        const esp_err_t e = uart_set_baudrate(_uartPort, baud);
        if (e != ESP_OK)
        {
            ESP_LOGE(TAG, "PX1105R UART: baud %lu refused (%s)", static_cast<unsigned long>(baud), esp_err_to_name(e));
            return false;
        }
        uart_flush_input(_uartPort);
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
        gpio_pullup_en(static_cast<gpio_num_t>(rx_pin_));   // idle-high while the receiver boots
        e = uart_driver_install(_uartPort, kUartRxBuf, kUartTxBuf, 16, &uart_events_, 0);
    }
    if (e != ESP_OK)
    {
        ESP_LOGE(TAG, "PX1105R UART: setup at %lu baud failed (%s)", static_cast<unsigned long>(baud),
                 esp_err_to_name(e));
        uart_events_ = nullptr;
        return false;
    }
    uart_set_rx_full_threshold(_uartPort, kRxFullThreshold);
    uart_baud_ = baud;
    return true;
}

size_t TR_GNSSReceiverPX1105RSerial::pump(uint32_t wait_ms, size_t max_bytes)
{
    if (!uart_is_driver_installed(_uartPort))
    {
        if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return 0;
    }
    const size_t want = std::min(max_bytes, sizeof(rx_buf_));
    const int n = uart_read_bytes(_uartPort, rx_buf_, want, pdMS_TO_TICKS(wait_ms));
    if (n <= 0) return 0;
    // When each byte arrived: the read time less the bytes after it (the
    // LC86G and mosaic drivers' rule: exact while the receiver is still
    // sending, late by the gap otherwise, never early)
    size_t behind = 0;
    uart_get_buffered_data_len(_uartPort, &behind);
    rd_us_ = nowUs();
    rd_n_ = static_cast<size_t>(n);
    rd_behind_ = behind;
    const uint32_t nmea_before = parser_.nmeaLines();
    parser_.feed(rx_buf_, rd_n_, *this);
    // each NMEA line (a factory receiver's 1 Hz burst is several in one read)
    const uint32_t ms = nowMs();
    for (uint32_t k = parser_.nmeaLines() - nmea_before; k > 0; --k) link_.onNmea(ms);
    return rd_n_;
}

void TR_GNSSReceiverPX1105RSerial::drainUartEvents()
{
    if (uart_events_ == nullptr) return;
    uart_event_t ev;
    while (xQueueReceive(uart_events_, &ev, 0) == pdTRUE)
    {
        if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) fifo_overflows_++;
    }
}

void TR_GNSSReceiverPX1105RSerial::runLink()
{
    for (int i = 0; i < 8; i++)
    {
        const skytraq::LinkAction a = link_.poll(nowMs());
        if (a.kind == skytraq::LinkAction::NONE) break;
        if (a.kind == skytraq::LinkAction::SET_BAUD)
        {
            (void)uartBegin(a.baud);
            parser_.reset();
        }
        else if (a.kind == skytraq::LinkAction::SEND && uart_is_driver_installed(_uartPort))
        {
            uart_write_bytes(_uartPort, reinterpret_cast<const char*>(a.bytes), a.len);
            // a command is under 50 bytes: wait for the wire so a baud change cannot cut one short
            uart_wait_tx_done(_uartPort, pdMS_TO_TICKS(50));
        }
    }
    if (link_.setups() != setups_logged_)
    {
        setups_logged_ = link_.setups();
        logSetup();
    }
}

int64_t TR_GNSSReceiverPX1105RSerial::arrivalUs(size_t index) const
{
    return rd_us_ - static_cast<int64_t>(static_cast<double>((rd_n_ - 1 - index) + rd_behind_) * byte_us_);
}

// ── begin() ─────────────────────────────────────────────────────────────

bool TR_GNSSReceiverPX1105RSerial::begin(uint8_t update_rate_hz, uint8_t GNSS_RX, uint8_t GNSS_TX,
                                         int8_t reset_n_pin, int8_t safeboot_n_pin)
{
    (void)reset_n_pin;     // J1 has no reset line: the GNSS rail's cycle is the reset
    (void)safeboot_n_pin;
    rx_pin_ = GNSS_RX;
    tx_pin_ = GNSS_TX;
    update_rate_hz_ = update_rate_hz;
    if (skytraq::rateCode(update_rate_hz_) < 0)
    {
        ESP_LOGW(TAG, "PX1105R: %u Hz is not one of its rates; using 20", update_rate_hz);
        update_rate_hz_ = 20;
    }

    if (ring_ == nullptr)
    {
        if ((reinterpret_cast<uintptr_t>(ring_mem_) & 0x3U) != 0)
            ESP_LOGE(TAG, "PX1105R: the log ring is not 32-bit aligned; frames will not be logged");
        else
            ring_ = xRingbufferCreateStatic(kRingBytes, RINGBUF_TYPE_NOSPLIT, ring_mem_, &ring_struct_);
    }
    if (clock_q_ == nullptr)
        clock_q_ = xQueueCreateStatic(kClockQDepth, sizeof(GnssClockData), clock_q_mem_, &clock_q_struct_);

    skytraq::LinkConfig cfg;
    cfg.target_baud = kTargetBaud;
    cfg.rate_hz = update_rate_hz_;
    cfg.subframes = static_cast<uint8_t>(0x0D | (TR_RAW_GLONASS ? 0x02 : 0x00));   // GPS, Galileo, BeiDou (+ GLONASS)
    link_.start(cfg, nowMs());
    setups_logged_ = 0;

    if (!begin_wait_)
    {
        runLink();
        ESP_LOGI(TAG, "Looking for the PX1105R (RX=%u TX=%u, no PPS) from the poll task, without waiting",
                 rx_pin_, tx_pin_);
        return true;
    }
    ESP_LOGI(TAG, "Looking for the PX1105R (RX=%u TX=%u, no PPS), up to %lu s...", rx_pin_, tx_pin_,
             static_cast<unsigned long>(begin_timeout_ms_ / 1000));
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
        // Found but not running yet (setting up, or no 0xDF after it):
        // the poll task carries on with it, and the FC still has its receiver.
        if (link_.state() != skytraq::LinkState::PROBE && link_.state() != skytraq::LinkState::VERIFY_BAUD)
        {
            ESP_LOGW(TAG, "PX1105R found at %lu baud but not streaming after %lu s (state %u, last failure: %s); "
                          "the poll task carries on",
                     static_cast<unsigned long>(link_.foundBaud()), static_cast<unsigned long>(begin_timeout_ms_ / 1000),
                     static_cast<unsigned>(link_.state()), link_.lastFailure());
            return true;
        }
        ESP_LOGE(TAG, "GNSS bring-up: no PX1105R answered in %lu s; continuing without GNSS",
                 static_cast<unsigned long>(begin_timeout_ms_ / 1000));
        return false;
    }
    return true;
}

void TR_GNSSReceiverPX1105RSerial::logSetup()
{
    ESP_LOGI(TAG, "PX1105R %s (#%lu): %lu baud (found at %lu%s), RTK kinematic base, airborne, %u Hz 0xE5/0xDF, "
                  "receiver %s; %lu refused, %lu readbacks off%s%s",
             link_.leftAlone() ? "found set up, left alone" : "set up",
             static_cast<unsigned long>(link_.setups()), static_cast<unsigned long>(link_.baud()),
             static_cast<unsigned long>(link_.foundBaud()), link_.foundNmea() ? ", NMEA" : ", binary",
             update_rate_hz_, link_.version()[0] ? link_.version() : "version not read",
             static_cast<unsigned long>(link_.refused()), static_cast<unsigned long>(link_.checksFailed()),
             link_.degraded() ? "; DEGRADED, last failure: " : "", link_.degraded() ? link_.lastFailure() : "");
}

// ── Frames ──────────────────────────────────────────────────────────────

void TR_GNSSReceiverPX1105RSerial::onFrame(const uint8_t* body, size_t len, size_t index)
{
    const int64_t end_us = arrivalUs(index);
    link_.onFrame(body, len, nowMs());
    n_frames_++;

    // The flight log's copy: the whole frame, verbatim. Never blocks; drops
    // count only once the flight loop drains the ring (the mosaic's rule).
    if (ring_ != nullptr)
    {
        const uint32_t t = static_cast<uint32_t>(end_us);
        std::memcpy(item_, &t, 4);
        const size_t n = skytraq::buildFrame(body, len, item_ + 4, sizeof(item_) - 4);
        if (n == 0)
            too_long_++;
        else if (xRingbufferSend(ring_, item_, 4 + n, 0) != pdTRUE && draining_.load(std::memory_order_relaxed))
            ring_dropped_++;
    }

    switch (body[0])
    {
    case skytraq::MSG_EXT_RAW:
        n_e5_++;
        if (raw_ != nullptr) rawE5(body, len, end_us);
        else
        {
            // the time sync runs without the raw outputs too (GNSS_CLOCK_MSG):
            // the header alone
            if (len >= 14 && body[1] == 1)
            {
                const uint16_t week = skytraq::be16(body + 3);
                const uint32_t tow_ms = skytraq::be32(body + 5);
                double dtr = 0.0;
                if (px1105r::biasAt(clk_, px1105r::rxMs(week, tow_ms), dtr))
                    time_sync_.addSample(px1105r::epochGpsS(week, tow_ms, dtr), end_us);
            }
        }
        break;
    case skytraq::MSG_RCV_STATE:
    {
        skytraq::RcvState s;
        if (!skytraq::decodeRcvState(body, len, s)) break;
        n_df_++;
        const bool cut = px1105r::noteRcvState(s, clk_);
        if (raw_ != nullptr)
        {
            if (cut)
            {
                raw_->lock.have_cut = true;
                raw_->lock.cut_ms = clk_.rx_ms;
            }
            rawDf(s, end_us);
        }
        publishFix(s, end_us);
        break;
    }
    case skytraq::MSG_SV_CH_STATUS:
        if (skytraq::decodeSvChStatus(body, len, channels_))
        {
            have_channels_ = true;
            last_e7_len_ = std::min(len, sizeof last_e7_);
            std::memcpy(last_e7_, body, last_e7_len_);
        }
        break;
    case skytraq::MSG_GPS_SUBFRAME:
    case skytraq::MSG_BDS_D1:
    case skytraq::MSG_GAL_PAGE:
    case skytraq::MSG_GLO_STRING:
        n_nav_++;
        if (raw_ != nullptr) rawNav(body, len);
        break;
    default:
        break;
    }

    // GNSS_CLOCK_MSG: the line each time it moves
    if (clock_q_ != nullptr && time_sync_.lineSeq() != clock_seq_seen_)
    {
        clock_seq_seen_ = time_sync_.lineSeq();
        lc86::GnssTimeSync::Snapshot snap;
        time_sync_.snapshot(snap);
        GnssClockData rec;
        lc86::gnssClockRecord(snap, static_cast<uint32_t>(end_us), rec);
        if (xQueueSend(clock_q_, &rec, 0) != pdTRUE) clock_dropped_++;
    }
}

void TR_GNSSReceiverPX1105RSerial::publishFix(const skytraq::RcvState& s, int64_t end_us)
{
    (void)end_us;
    gnssDataFromRcvState(s, have_channels_ ? channels_.used : 0, data_);
    // MCU sample time, as the other drivers stamp it; the freshness gates key off it
    data_.time_us = static_cast<uint32_t>(nowUs());
    new_pvt_ = true;
    if (data_.fix_mode >= 2) n_fix_++;
}

// ── Poll path ───────────────────────────────────────────────────────────

bool TR_GNSSReceiverPX1105RSerial::pollNewPVT(GNSSData& gnss_data)
{
    size_t drained = 0;
    while (drained < kMaxBytesPerPoll)
    {
        const size_t n = pump(0, kMaxBytesPerPoll - drained);
        if (n == 0) break;
        drained += n;
    }
    drainUartEvents();
    runLink();
    logStatus();
    if (!new_pvt_) return false;
    new_pvt_ = false;
    getGNSSData(gnss_data);
    return true;
}

void TR_GNSSReceiverPX1105RSerial::getGNSSData(GNSSData& gnss_data)
{
    gnss_data = data_;
}

bool TR_GNSSReceiverPX1105RSerial::pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us)
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
    if (!fits) short_buffer_++;
    return fits;
}

bool TR_GNSSReceiverPX1105RSerial::takeClockLine(GnssClockData& out)
{
    return clock_q_ != nullptr && xQueueReceive(clock_q_, &out, 0) == pdTRUE;
}

void TR_GNSSReceiverPX1105RSerial::logStatus()
{
    const int64_t now = nowUs();
    if (status_us_ == 0)
    {
        status_us_ = now;
        return;
    }
    const double span_s = static_cast<double>(now - status_us_) * 1.0e-6;
    if (span_s < 30.0) return;
    status_us_ = now;
    ESP_LOGI(TAG, "PX1105R: %.1f 0xE5/s, %.1f 0xDF/s (%.1f with a fix), %.1f subframes/s; %lu frames, checksum "
                  "fails %lu, bad lengths %lu, NMEA lines %lu; log ring drops %lu, too long %lu; UART overflows %lu; "
                  "link %s, relinks %lu; raw period %u ms; used %u of %u tracked; poll task stack %lu B free",
             n_e5_ / span_s, n_df_ / span_s, n_fix_ / span_s, n_nav_ / span_s, static_cast<unsigned long>(n_frames_),
             static_cast<unsigned long>(parser_.badChecksums()), static_cast<unsigned long>(parser_.badLengths()),
             static_cast<unsigned long>(parser_.nmeaLines()), static_cast<unsigned long>(ring_dropped_),
             static_cast<unsigned long>(too_long_), static_cast<unsigned long>(fifo_overflows_),
             link_.running() ? (link_.degraded() ? "running DEGRADED" : "running") : "setting up",
             static_cast<unsigned long>(link_.relinks()),
             link_.rawPeriodMs(), channels_.used, channels_.tracked,
             static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
    logChannels();
    ESP_LOGI(TAG, "PX1105R clock: bias %.4f ms, drift %+.3f ppm, %lu jumps, %lu restarts; arrival line %s, MCU %+.2f ppm, "
                  "rms %.3f ms, late90 %.2f ms, %lu restarts; clock records dropped %lu",
             clk_.bias_ms, clk_.drift_ms_per_s * 1.0e3, static_cast<unsigned long>(clk_.jumps),
             static_cast<unsigned long>(clk_.restarts), time_sync_.valid() ? "valid" : "not valid yet",
             time_sync_.driftPpm(), time_sync_.envelopeRmsMs(), time_sync_.lateness90Ms(),
             static_cast<unsigned long>(time_sync_.restarts()), static_cast<unsigned long>(clock_dropped_));
    if (raw_ != nullptr)
    {
        const RawStats& st = raw_->st;
        ESP_LOGI(TAG, "PX1105R raw: %lu epochs, %lu handed over (%lu untimed), %lu off the grid, %lu empty, %lu dropped, "
                      "%lu slips; ephemerides GPS %lu GAL %lu BDS %lu GLO %lu (dropped %lu/%lu/%lu/%lu), refused %lu, "
                      "ionospheres %lu; "
                      "fixes %lu (dropped %lu)",
                 static_cast<unsigned long>(st.epochs), static_cast<unsigned long>(st.published),
                 static_cast<unsigned long>(st.untimed), static_cast<unsigned long>(st.decimated),
                 static_cast<unsigned long>(st.empty), static_cast<unsigned long>(st.dropped),
                 static_cast<unsigned long>(st.slips), static_cast<unsigned long>(st.eph_queued[0]),
                 static_cast<unsigned long>(st.eph_queued[1]), static_cast<unsigned long>(st.eph_queued[2]),
                 static_cast<unsigned long>(st.eph_queued[3]), static_cast<unsigned long>(st.eph_dropped[0]),
                 static_cast<unsigned long>(st.eph_dropped[1]), static_cast<unsigned long>(st.eph_dropped[2]),
                 static_cast<unsigned long>(st.eph_dropped[3]), static_cast<unsigned long>(st.nav_refused),
                 static_cast<unsigned long>(st.ions), static_cast<unsigned long>(st.fixes),
                 static_cast<unsigned long>(st.fixes_dropped));
    }
    n_e5_ = n_df_ = n_nav_ = n_fix_ = 0;
}

// Each signal the last 0xE7 lists with a C/N0 above 0: G GPS, S SBAS, R GLONASS,
// E Galileo, J QZSS, C BeiDou, I NavIC, then the satellite; "/5" the L5 band's
// signal; "*" used in the fix.
void TR_GNSSReceiverPX1105RSerial::logChannels()
{
    if (last_e7_len_ < 4) return;
    static constexpr char kLetter[] = "GSREJCI?";
    char line[480];
    size_t n = 0;
    unsigned shown = 0;
    const size_t listed = last_e7_[3];
    for (size_t i = 0; i < listed && 4 + 7 * (i + 1) <= last_e7_len_; ++i)
    {
        const uint8_t* r = last_e7_ + 4 + 7 * i;
        const int cn0 = static_cast<int8_t>(r[5]);
        if (cn0 <= 0) continue;
        const int w = std::snprintf(line + n, sizeof line - n, "%s%c%02u%s %d%s", shown ? ", " : "",
                                    kLetter[std::min<uint8_t>(r[1] & 0x07, 7)], r[2],
                                    (r[1] >> 4) == skytraq::kSigL5 ? "/5" : "", cn0, (r[6] & 0x10) ? "*" : "");
        if (w < 0 || static_cast<size_t>(w) >= sizeof line - n) break;
        n += static_cast<size_t>(w);
        shown++;
    }
    ESP_LOGI(TAG, "PX1105R C/N0 (dB-Hz, * in the fix), %u of %u listed: %s", shown, static_cast<unsigned>(listed),
             shown ? line : "none above 0");
}

// ── Raw outputs ─────────────────────────────────────────────────────────

bool TR_GNSSReceiverPX1105RSerial::enableRawOutputs()
{
    if (raw_ != nullptr) return true;
    // Internal RAM: the poll task works in it at 20 Hz.
    void* mem = heap_caps_malloc(sizeof(RawState), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (mem == nullptr)
    {
        ESP_LOGE(TAG, "PX1105R raw: no memory for the raw outputs (%u B)", static_cast<unsigned>(sizeof(RawState)));
        return false;
    }
    RawState* r = new (mem) RawState();
    r->epoch_q = xQueueCreate(kRawEpochQDepth, sizeof(gnss_raw::TimedRawEpoch));
    r->eph_q = xQueueCreate(kEphQDepth, sizeof(gps_eph_t));
    r->fix_q = xQueueCreate(kFixQDepth, sizeof(gnss_raw::TimedFix));
    if (r->epoch_q == nullptr || r->eph_q == nullptr || r->fix_q == nullptr)
    {
        ESP_LOGE(TAG, "PX1105R raw: no memory for the queues");
        if (r->epoch_q != nullptr) vQueueDelete(r->epoch_q);
        if (r->eph_q != nullptr) vQueueDelete(r->eph_q);
        if (r->fix_q != nullptr) vQueueDelete(r->fix_q);
        r->~RawState();
        heap_caps_free(mem);
        return false;
    }
    // The subframes' assembly in PSRAM where there is some: a few frames a
    // second, never on a deadline
    void* nav_mem = heap_caps_malloc(sizeof(skytraq::NavAssembler), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (nav_mem == nullptr) nav_mem = heap_caps_malloc(sizeof(skytraq::NavAssembler), MALLOC_CAP_8BIT);
    if (nav_mem == nullptr)
    {
        ESP_LOGE(TAG, "PX1105R raw: no memory for the subframes (%u B)", static_cast<unsigned>(sizeof(skytraq::NavAssembler)));
        vQueueDelete(r->epoch_q);
        vQueueDelete(r->eph_q);
        vQueueDelete(r->fix_q);
        r->~RawState();
        heap_caps_free(mem);
        return false;
    }
    r->nav = new (nav_mem) skytraq::NavAssembler();
    r->nav->reset();
#if TR_RAW_L5
    // the L5-band codes' bridge, PSRAM where there is some (a few KB, once an epoch); without it the L5
    // band's codes are left as they come (never fused) and its Dopplers still go
    void* br_mem = heap_caps_malloc(sizeof(gnss_raw::CodeBridge), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (br_mem == nullptr) br_mem = heap_caps_malloc(sizeof(gnss_raw::CodeBridge), MALLOC_CAP_8BIT);
    if (br_mem != nullptr)
        r->bridge = new (br_mem) gnss_raw::CodeBridge();
    else
        ESP_LOGW(TAG, "PX1105R raw: no memory for the L5 code bridge (%u B): L5-band codes never fused",
                 static_cast<unsigned>(sizeof(gnss_raw::CodeBridge)));
#endif
    r->st.enabled = true;
    raw_ = r;   // before begin(): the poll task does not exist yet
    ESP_LOGI(TAG, "PX1105R raw: outputs on -- %u epochs (%u B), %u ephemerides (%u B), %u B of state, "
                  "%u B of subframes in %s",
             static_cast<unsigned>(kRawEpochQDepth),
             static_cast<unsigned>(kRawEpochQDepth * sizeof(gnss_raw::TimedRawEpoch)),
             static_cast<unsigned>(kEphQDepth), static_cast<unsigned>(kEphQDepth * sizeof(gps_eph_t)),
             static_cast<unsigned>(sizeof(RawState)), static_cast<unsigned>(sizeof(skytraq::NavAssembler)),
             esp_ptr_external_ram(nav_mem) ? "PSRAM" : "internal RAM");
    return true;
}

void TR_GNSSReceiverPX1105RSerial::setRawEpochPeriodMs(uint32_t period_ms)
{
    if (raw_ == nullptr) return;
    raw_->period_ms.store(period_ms, std::memory_order_relaxed);
    raw_->st.period_ms = period_ms;
}

bool TR_GNSSReceiverPX1105RSerial::setRawDopplerPairing(const mosaic::MosaicDopplerPairConfig& c)
{
    if (raw_ == nullptr) return false;
    if (raw_->pairing.load(std::memory_order_acquire) != nullptr) return true;
    void* mem = heap_caps_malloc(sizeof(RawState::Pairing), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (mem == nullptr)
    {
        ESP_LOGE(TAG, "PX1105R raw: no memory for the Doppler pairs (%u B): every epoch's own Dopplers",
                 static_cast<unsigned>(sizeof(RawState::Pairing)));
        return false;
    }
    RawState::Pairing* p = new (mem) RawState::Pairing();
    p->cfg = c;
    raw_->pairing.store(p, std::memory_order_release);
    ESP_LOGI(TAG, "PX1105R raw: Dopplers in pairs -- each grid epoch's with the epoch %lu ms before it, at most "
                  "%.1f m/s apart, %u B of state",
             static_cast<unsigned long>(c.dt_ms), c.max_diff_mps, static_cast<unsigned>(sizeof(RawState::Pairing)));
    return true;
}

bool TR_GNSSReceiverPX1105RSerial::takeRawEpoch(gnss_raw::TimedRawEpoch& out)
{
    if (raw_ == nullptr) return false;
    raw_->taking_epochs.store(true, std::memory_order_relaxed);
    return xQueueReceive(raw_->epoch_q, &out, 0) == pdTRUE;
}

bool TR_GNSSReceiverPX1105RSerial::takeEphemeris(gps_eph_t& out)
{
    if (raw_ == nullptr) return false;
    raw_->taking_eph.store(true, std::memory_order_relaxed);
    return xQueueReceive(raw_->eph_q, &out, 0) == pdTRUE;
}

bool TR_GNSSReceiverPX1105RSerial::takeFix(gnss_raw::TimedFix& out)
{
    if (raw_ == nullptr) return false;
    raw_->taking_fix.store(true, std::memory_order_relaxed);
    return xQueueReceive(raw_->fix_q, &out, 0) == pdTRUE;
}

bool TR_GNSSReceiverPX1105RSerial::broadcastIono(int8_t raw[8], uint16_t& gps_day) const
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

void TR_GNSSReceiverPX1105RSerial::rawStats(RawStats& out) const
{
    out = (raw_ != nullptr) ? raw_->st : RawStats{};   // the line's numbers copied in by the poll task (rawE5)
}

bool TR_GNSSReceiverPX1105RSerial::rawPairStats(RawPairStats& out) const
{
    const RawState::Pairing* p = raw_ != nullptr ? raw_->pairing.load(std::memory_order_acquire) : nullptr;
    out = p != nullptr ? p->st : RawPairStats{};
    return p != nullptr;
}

void TR_GNSSReceiverPX1105RSerial::rawE5(const uint8_t* body, size_t len, int64_t end_us)
{
    RawState& r = *raw_;
    if (!skytraq::decodeRawE5(body, len, r.e5)) return;
    r.st.epochs++;
    const int64_t rx_ms = px1105r::rxMs(r.e5.week, r.e5.tow_ms);

    // the time sync's sample: this epoch's GPS instant against its arrival
    double dtr = 0.0;
    const bool biased = px1105r::biasAt(clk_, rx_ms, dtr);
    const double gps_s = px1105r::epochGpsS(r.e5.week, r.e5.tow_ms, dtr);
    if (biased) time_sync_.addSample(gps_s, end_us);
    // the line's numbers for rawStats, from this task (the only one that touches time_sync_)
    r.st.line_valid = time_sync_.valid();
    r.st.line_rms_ms = static_cast<float>(time_sync_.envelopeRmsMs());
    r.st.late90_ms = static_cast<float>(time_sync_.lateness90Ms());
    r.st.line_restarts = time_sync_.restarts();

    RawState::Pairing* const pairing = r.pairing.load(std::memory_order_acquire);
    gnss_raw::TimedRawEpoch& e = r.scratch;
    const bool on_grid = px1105r::onFuseGrid(r.e5.tow_ms, r.period_ms.load(std::memory_order_relaxed));
    const int64_t t_adapt = nowUs();
#if TR_RAW_GLONASS
    // each GLONASS slot's frequency number, for its ephemerides (the strings carry none)
    for (int i = 0; i < r.e5.n; ++i)
        if (r.e5.cell[i].gnss == skytraq::kGnssGlo)
            r.nav->noteGloFreq(r.e5.cell[i].svid, static_cast<int>(r.e5.cell[i].glo_freq) - 7);
#endif
    const int cells = px1105r::rawEpochFromE5(r.e5, r.lock, e.epoch, TR_RAW_GLONASS != 0, TR_RAW_L5 != 0);
    if (r.bridge != nullptr) r.bridge->apply(e.epoch);   // every epoch: its differences kept at 20 Hz
    r.st.slips = r.lock.slips;
    if (!on_grid)
    {
        // Off the grid: only its Dopplers, for the next grid epoch's pairs
        if (pairing != nullptr)
        {
            mosaic::dopplerSetFromRawEpoch(e.epoch, rx_ms, pairing->prev);
            const uint32_t us = static_cast<uint32_t>(nowUs() - t_adapt);
            RawPairStats& ps = pairing->st;
            ps.reads++;
            ps.read_us_sum += us;
            if (us > ps.read_us_max) ps.read_us_max = us;
        }
        r.st.decimated++;
        return;
    }
    e.rr_paired = 0;
    if (pairing != nullptr)
    {
        const int64_t t_avg = nowUs();
        const uint32_t us = static_cast<uint32_t>(t_avg - t_adapt);
        RawPairStats& ps = pairing->st;
        ps.adapts++;
        ps.adapt_us_sum += us;
        if (us > ps.adapt_us_max) ps.adapt_us_max = us;
        // a clock jump between the two: no pairs (MosaicRawEpoch.h's rule)
        const bool cut = r.lock.have_cut && r.lock.cut_ms > rx_ms - static_cast<int64_t>(pairing->cfg.dt_ms) &&
                         r.lock.cut_ms <= rx_ms;
        if (cells > 0)
        {
            const int n = mosaic::averageDopplerPairs(e.epoch, rx_ms, cut, pairing->prev, pairing->cfg, ps.pair);
            e.rr_paired = n > 0 ? 1 : 0;
        }
        const uint32_t avg_us = static_cast<uint32_t>(nowUs() - t_avg);
        if (avg_us > ps.avg_us_max) ps.avg_us_max = avg_us;
        pairing->prev.valid = false;   // one grid epoch's mate only
    }
    if (cells == 0)
    {
        r.st.empty++;
        return;
    }
    int64_t mcu = 0;
    const bool timed = biased && time_sync_.valid() && time_sync_.mcuAt(gps_s, mcu);
    e.arrival_us = end_us;
    e.gps_week = r.e5.week;
    e.mcu_us = timed ? mcu : 0;
    e.tag_dtr_ms = timed ? static_cast<float>(dtr) : 0.0f;
    if (xQueueSend(r.epoch_q, &e, 0) == pdTRUE)
    {
        r.st.published++;
        if (!timed) r.st.untimed++;
    }
    else if (r.taking_epochs.load(std::memory_order_relaxed))
    {
        r.st.dropped++;
    }
}

void TR_GNSSReceiverPX1105RSerial::rawDf(const skytraq::RcvState& s, int64_t end_us)
{
    RawState& r = *raw_;
    r.st.clock_jumps = clk_.jumps;
    r.st.clock_restarts = clk_.restarts;
    r.st.bias_ms = static_cast<float>(clk_.bias_ms);
    r.st.drift_ppm = static_cast<float>(clk_.drift_ms_per_s * 1.0e3);
    if (s.nav_state < skytraq::kNavFix2d) return;
    r.st.fixes++;
    gnss_raw::TimedFix f;
    double h;
    ecefToLla(s.ecef_m, f.lat_rad, f.lon_rad, h);
    f.alt_m = h;   // already above the ellipsoid
    const double sl = std::sin(f.lat_rad), cl = std::cos(f.lat_rad), so = std::sin(f.lon_rad), co = std::cos(f.lon_rad);
    const double vx = s.vel_mps[0], vy = s.vel_mps[1], vz = s.vel_mps[2];
    f.vel_ned[0] = static_cast<float>(-sl * co * vx - sl * so * vy + cl * vz);
    f.vel_ned[1] = static_cast<float>(-so * vx + co * vy);
    f.vel_ned[2] = static_cast<float>(-(cl * co * vx + cl * so * vy + sl * vz));
    f.fix_mode = s.nav_state == skytraq::kNavFix2d ? 2 : 3;
    f.num_sats = have_channels_ ? channels_.used : 0;
    f.arrival_us = end_us;
    f.gps_s = s.week * 604800.0 + s.tow_s;   // 0xDF's time is GPS time
    int64_t mcu = 0;
    f.mcu_us = time_sync_.valid() && time_sync_.mcuAt(f.gps_s, mcu) ? mcu : 0;
    if (xQueueSend(r.fix_q, &f, 0) != pdTRUE && r.taking_fix.load(std::memory_order_relaxed)) r.st.fixes_dropped++;
}

void TR_GNSSReceiverPX1105RSerial::rawNav(const uint8_t* body, size_t len)
{
    RawState& r = *raw_;
    if (!skytraq::isNavMessage(body, len)) return;
    // RawStats' index: 0 GPS, 1 Galileo, 2 BeiDou, 3 GLONASS
    const int sys = body[0] == skytraq::MSG_GPS_SUBFRAME ? 0
                  : body[0] == skytraq::MSG_GAL_PAGE     ? 1
                  : body[0] == skytraq::MSG_GLO_STRING   ? 3
                                                         : 2;
    r.st.nav_frames[sys]++;
    auto queue = [&](const gps_eph_t& eph) {
        const int k = eph.sys == GNSS_SYS_GPS ? 0
                    : eph.sys == GNSS_SYS_GAL ? 1
                    : eph.sys == GNSS_SYS_GLO ? 3
                                              : 2;
        if (xQueueSend(r.eph_q, &eph, 0) == pdTRUE)
            r.st.eph_queued[k]++;
        else if (r.taking_eph.load(std::memory_order_relaxed))
            r.st.eph_dropped[k]++;
    };
    gps_eph_t eph;
    int8_t iono[8];
    // the receiver's GPS week, from its own clock (0 until a 0xDF has come: a
    // GPS ephemeris then waits as NO_WEEK, completed below once it is known)
    const int week = clk_.have ? static_cast<int>(clk_.rx_ms / 604800000LL) : 0;
    // and its time, which places a GLONASS set's tb on its day (0: unknown, the strings kept)
    const double gps_s = clk_.have ? static_cast<double>(clk_.rx_ms) * 1.0e-3 : 0.0;
    const skytraq::NavStatus st = r.nav->add(body, len, week, eph, iono, gps_s);
    switch (st)
    {
    case skytraq::NavStatus::EPHEMERIS:
        queue(eph);
        break;
    case skytraq::NavStatus::IONO:
    {
        // every satellite's page 18: the latest wins (the GNSS task's rule
        // chooses between it and the app's by their days)
        const uint16_t day = r.nav->ionoGpsDay();
        if (day != 0)
        {
            portENTER_CRITICAL(&raw_mux_);
            std::memcpy(r.iono_raw, iono, sizeof r.iono_raw);
            r.iono_day = day;
            r.iono_have = true;
            portEXIT_CRITICAL(&raw_mux_);
            r.st.ions++;
        }
        break;
    }
    case skytraq::NavStatus::PART:
    case skytraq::NavStatus::UNUSED:
    case skytraq::NavStatus::NO_WEEK:
    case skytraq::NavStatus::NO_TIME:
        break;
    default:
        r.st.nav_refused++;
        break;
    }
    // GPS ephemerides held for the week
    if (week != 0)
    {
        while (r.nav->takePending(week, eph) == skytraq::NavStatus::EPHEMERIS)
            queue(eph);
    }
}
