// gnss_auto_driver.cpp — see gnss_auto_driver.h. The UART, NVS and timing
// around gnss_detect.cpp, which decides everything (host-tested).
#if defined(TR_GNSS_DRIVER_AUTO) && TR_GNSS_DRIVER_AUTO

#include "gnss_auto_driver.h"

#include <sdkconfig.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <TR_NVS.h>

// The mosaic streams ~66 kB/s at 2 Mbaud (the gnss-sdr-dev board up to
// ~55 kB/s), which fills the UART's 128-byte RX FIFO in 0.64 ms: less than a
// flash write keeps an interrupt that needs the cache waiting. sdkconfig.defaults.mosaic puts the UART interrupt in IRAM;
// a build dir made before this image had the overlay keeps its old sdkconfig.
#if !CONFIG_UART_ISR_IN_IRAM
#error "TR_GNSS_DRIVER_AUTO needs CONFIG_UART_ISR_IN_IRAM (sdkconfig.defaults.mosaic): use a fresh build dir, or delete its sdkconfig"
#endif

static const char* TAG = "GNSS";

namespace
{

constexpr size_t kRxBuf = 8192;   // ~120 ms of a set-up mosaic; the loop below reads every few ms
constexpr size_t kTxBuf = 256;

// The gnss-sdr-dev board's driver is read as often as the mosaic's: 1.4 kB a
// read at its 55 kB/s, its 16 kB UART ring 0.3 s of the stream.
constexpr uint8_t kSdrPollRateHz = 20;
// The PX1105R's 0xE5 and 0xDF at 20 Hz, as its own image sets it up
// (config.h GNSS_UPDATE_RATE there); its poll task twice that.
constexpr uint8_t kPx1105rRateHz = 20;

constexpr const char* kHintNamespace = "gnssauto";
constexpr const char* kHintKey = "rx";

inline uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

const char* nameOr(uint8_t type, const char* none)
{
    const char* n = gnssTypeName(type);
    return n[0] != '\0' ? n : none;
}

}  // namespace

TR_GNSSAuto::TR_GNSSAuto(uart_port_t uart_port)
    : port_(uart_port), ublox_(uart_port), mosaic_(uart_port), sdr_(uart_port)
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    , px_(uart_port)
#endif
{
}

// ── NVS ─────────────────────────────────────────────────────────────────

uint8_t TR_GNSSAuto::rememberedType()
{
    if (!hint_loaded_)
    {
        hint_loaded_ = true;
        hint_ = GNSS_TYPE_NONE;
        Preferences prefs;
        if (prefs.begin(kHintNamespace, true))   // absent until a receiver is first found
        {
            hint_ = prefs.getUChar(kHintKey, GNSS_TYPE_NONE);
            prefs.end();
        }
    }
    return hint_;
}

void TR_GNSSAuto::storeHint(uint8_t type)
{
    if (type == rememberedType()) return;   // NVS is flash, and this runs every boot
    Preferences prefs;
    if (!prefs.begin(kHintNamespace, false))
    {
        ESP_LOGW(TAG, "GNSS auto: could not open NVS to remember the receiver");
        return;
    }
    prefs.putUChar(kHintKey, type);
    prefs.end();
    hint_ = type;
    ESP_LOGI(TAG, "GNSS auto: remembered %s for the next boot", nameOr(type, "no receiver"));
}

void TR_GNSSAuto::ubloxLastLink(uint32_t& baud, bool& swapped) const
{
    // TR_GNSSReceiverUBlox_Serial begin()'s "gnsslink": baud, rx, tx.
    baud = 0;
    swapped = false;
    Preferences prefs;
    if (!prefs.begin("gnsslink", true)) return;
    baud = prefs.getUInt("baud", 0U);
    const uint8_t rx = prefs.getUChar("rx", 0xFF);
    const uint8_t tx = prefs.getUChar("tx", 0xFF);
    prefs.end();
    swapped = rx_pin_ != tx_pin_ && rx == tx_pin_ && tx == rx_pin_;
}

// ── UART ────────────────────────────────────────────────────────────────

bool TR_GNSSAuto::uartOpen(uint32_t baud)
{
    if (uart_is_driver_installed(port_))
    {
        if (uart_set_baudrate(port_, baud) != ESP_OK) return false;
        uart_flush_input(port_);   // heard at the old baud
        return true;
    }
    uart_config_t cfg = {};
    cfg.baud_rate  = static_cast<int>(baud);
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    esp_err_t e = uart_param_config(port_, &cfg);
    if (e == ESP_OK)
    {
        e = uart_set_pin(port_, static_cast<int>(tx_pin_), static_cast<int>(rx_pin_),
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (e == ESP_OK)
    {
        // Idle-high while nothing drives RX, as both drivers hold it.
        gpio_pullup_en(static_cast<gpio_num_t>(rx_pin_));
        e = uart_driver_install(port_, kRxBuf, kTxBuf, 0, nullptr, 0);
    }
    if (e != ESP_OK)
    {
        ESP_LOGE(TAG, "GNSS auto: UART setup at %lu baud failed (%s)", static_cast<unsigned long>(baud),
                 esp_err_to_name(e));
        return false;
    }
    return true;
}

void TR_GNSSAuto::uartClose()
{
    // Each driver installs the UART its own way (the mosaic's and the
    // gnss-sdr-dev board's with an event queue and a 16 kB ring), and they
    // only re-rate one they find installed, so none is left for them.
    if (uart_is_driver_installed(port_)) uart_driver_delete(port_);
}

void TR_GNSSAuto::runPlan(const gnss_detect::Plan& plan)
{
    // From a UART of our own: after the u-blox driver's search the port is
    // still installed the way its last attempt left it, which can be the
    // swapped-wires one (RX on GPIO4, TX on GPIO3), and uartOpen() would
    // only re-rate it.
    uartClose();
    detector_.start(plan, nowMs());
    uint8_t buf[256];
    while (!detector_.done())
    {
        for (int i = 0; i < 4; i++)
        {
            const gnss_detect::Action a = detector_.poll(nowMs());
            if (a.kind == gnss_detect::Action::NONE) break;
            if (a.kind == gnss_detect::Action::SET_BAUD)
            {
                // A port that will not come up at this baud just hears nothing
                // for the probe's window.
                (void)uartOpen(a.baud);
            }
            else if (uart_is_driver_installed(port_))
            {
                uart_write_bytes(port_, a.data, a.len);
                uart_wait_tx_done(port_, pdMS_TO_TICKS(20));   // 8 bytes at 9600 baud: 8.3 ms
            }
        }
        if (detector_.done()) break;
        if (!uart_is_driver_installed(port_))
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        // Blocks up to 5 ms: the probes' windows are 106-412 ms.
        const int n = uart_read_bytes(port_, buf, sizeof(buf), pdMS_TO_TICKS(5));
        if (n > 0) detector_.feed(buf, static_cast<size_t>(n), nowMs());
    }
}

// ── begin() ─────────────────────────────────────────────────────────────

bool TR_GNSSAuto::begin(uint8_t update_rate_hz,
                        uint8_t GNSS_RX,
                        uint8_t GNSS_TX,
                        int8_t reset_n_pin,
                        int8_t safeboot_n_pin)
{
    rx_pin_ = GNSS_RX;
    tx_pin_ = GNSS_TX;
    ublox_rate_hz_ = update_rate_hz;
    type_ = GNSS_TYPE_NONE;

    // A u-blox that kept its power since the flight computer set it up is
    // still at 460800: anything but a power-on (an OTA, a crash) keeps the
    // receiver's rail up.
    const esp_reset_reason_t rr = esp_reset_reason();
    const bool kept_power = !(rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT);

#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    // A restored flight: no probes, the raw receiver it flew with at once
    // (the u-blox, or nothing known, take the path below as always)
    if (!begin_wait_ && beginHintedRaw(rememberedType(), GNSS_RX, GNSS_TX, reset_n_pin, safeboot_n_pin))
        return type_ != GNSS_TYPE_NONE;
    constexpr bool kLookForPx1105r = true;
#else
    constexpr bool kLookForPx1105r = false;
#endif
    uint32_t ublox_baud = 0;
    bool ublox_swapped = false;
    ubloxLastLink(ublox_baud, ublox_swapped);
    gnss_detect::BringUp up;
    up.start(rememberedType(), ublox_baud, kept_power, ublox_swapped, kLookForPx1105r);
    ESP_LOGI(TAG, "GNSS auto: which receiver is on J1 (RX=%u TX=%u)? Last boot: %s",
             rx_pin_, tx_pin_, nameOr(rememberedType(), "nothing known"));
    if (up.plan().n == 0)
    {
        ESP_LOGI(TAG, "GNSS auto: the u-blox driver last found its receiver on swapped wires: "
                      "straight to it, no probes");
    }

    while (up.step() != gnss_detect::BringUp::Step::DONE)
    {
        switch (up.step())
        {
            case gnss_detect::BringUp::Step::DETECT:
            case gnss_detect::BringUp::Step::SECOND_LOOK:
            {
                const bool second = up.step() == gnss_detect::BringUp::Step::SECOND_LOOK;
                runPlan(up.plan());
                uartClose();
                if (detector_.found() != GNSS_TYPE_NONE)
                {
                    ESP_LOGI(TAG, "GNSS auto: %s on J1 (%s at %lu baud, %lu ms, %lu probe%s)",
                             gnssTypeName(detector_.found()), gnss_detect::proofName(detector_.proof()),
                             static_cast<unsigned long>(detector_.foundBaud()),
                             static_cast<unsigned long>(detector_.elapsedMs()),
                             static_cast<unsigned long>(detector_.probesRun()),
                             detector_.probesRun() == 1 ? "" : "s");
                }
                else if (second)
                {
                    ESP_LOGW(TAG, "GNSS auto: no mosaic-G5 or gnss-sdr-dev board either (%lu ms)",
                             static_cast<unsigned long>(detector_.elapsedMs()));
                }
                else if (up.plan().n > 0)
                {
                    ESP_LOGW(TAG, "GNSS auto: no receiver answered the probes in %lu ms (%lu probes)%s",
                             static_cast<unsigned long>(detector_.elapsedMs()),
                             static_cast<unsigned long>(detector_.probesRun()),
                             up.plan().fallback == GNSS_TYPE_UBLOX
                                 ? "; the u-blox driver searches as it always has"
                             : up.plan().fallback == GNSS_TYPE_SDR
                                 ? "; the gnss-sdr-dev board that streamed last boot sends from its first fix: "
                                   "listening for it"
                             : up.plan().fallback == GNSS_TYPE_PX1105R
                                 ? "; the PX1105R that answered last boot gets its driver, which goes on looking"
                                 : "");
                }
                up.detected(detector_.found());
                // Before the driver runs: a crash inside it must find the
                // receiver remembered (see the header).
                storeHint(up.hint());
                if (up.step() == gnss_detect::BringUp::Step::DONE && up.plan().fallback == GNSS_TYPE_NONE && !second)
                {
                    ESP_LOGW(TAG, "GNSS auto: the mosaic-G5 that was here last boot did not answer in %lu s; "
                                  "continuing without GNSS",
                             static_cast<unsigned long>(gnss_detect::kBudgetMosaicMs / 1000));
                }
                break;
            }
            case gnss_detect::BringUp::Step::BEGIN_UBLOX:
            {
                const bool ok = ublox_.begin(ublox_rate_hz_, GNSS_RX, GNSS_TX, reset_n_pin, safeboot_n_pin);
                up.driverResult(ok);
                if (!ok && up.step() == gnss_detect::BringUp::Step::SECOND_LOOK)
                {
                    ESP_LOGW(TAG, "GNSS auto: the u-blox driver found nothing; a last look for a mosaic-G5");
                }
                storeHint(up.hint());
                break;
            }
            case gnss_detect::BringUp::Step::BEGIN_MOSAIC:
            {
                mosaic_.setBeginTimeoutMs(up.mosaicTimeoutMs());
                const bool ok = mosaic_.begin(mosaic_rate_hz_, GNSS_RX, GNSS_TX, reset_n_pin, safeboot_n_pin);
                up.driverResult(ok);
                storeHint(up.hint());
                break;
            }
            case gnss_detect::BringUp::Step::BEGIN_SDR:
            {
                const bool ok = sdr_.begin(kSdrPollRateHz, GNSS_RX, GNSS_TX, reset_n_pin, safeboot_n_pin);
                up.driverResult(ok);
                storeHint(up.hint());
                break;
            }
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
            case gnss_detect::BringUp::Step::BEGIN_PX1105R:
            {
                const bool ok = px_.begin(kPx1105rRateHz, GNSS_RX, GNSS_TX, reset_n_pin, safeboot_n_pin);
                up.driverResult(ok);
                storeHint(up.hint());
                break;
            }
#endif
            default:
                break;
        }
    }

    type_ = up.active();
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    if (rawReceiver())
    {
        ESP_LOGI(TAG, "GNSS auto: flying the raw-GNSS filter on the %s", gnssTypeName(type_));
    }
    else
#endif
    if (type_ == GNSS_TYPE_SDR)
    {
        ESP_LOGI(TAG, "GNSS auto: logging the gnss-sdr-dev board's RTCM3; the FC navigates without GNSS");
    }
    else if (type_ != GNSS_TYPE_NONE)
    {
        ESP_LOGI(TAG, "GNSS auto: flying the %s", gnssTypeName(type_));
    }
    return type_ != GNSS_TYPE_NONE;
}

// ── Calls after begin() ─────────────────────────────────────────────────

uint8_t TR_GNSSAuto::updateRateHz() const
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_rate_hz_ > 20 ? 20 : mosaic_rate_hz_;
    if (type_ == GNSS_TYPE_SDR) return kSdrPollRateHz;
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    if (type_ == GNSS_TYPE_PX1105R) return kPx1105rRateHz;
#endif
    return ublox_rate_hz_;
}

uint8_t TR_GNSSAuto::otpState() const
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.otpState();
    if (type_ == GNSS_TYPE_SDR) return sdr_.otpState();
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    if (type_ == GNSS_TYPE_PX1105R) return px_.otpState();
#endif
    return ublox_.otpState();
}

void TR_GNSSAuto::getGNSSData(GNSSData& data)
{
    if (type_ == GNSS_TYPE_MOSAIC)
        mosaic_.getGNSSData(data);
    else if (type_ == GNSS_TYPE_SDR)
        sdr_.getGNSSData(data);
    else if (type_ == GNSS_TYPE_UBLOX)
        ublox_.getGNSSData(data);
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    else if (type_ == GNSS_TYPE_PX1105R)
        px_.getGNSSData(data);
#endif
    else
        data = GNSSData{};
}

bool TR_GNSSAuto::pollNewPVT(GNSSData& data)
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.pollNewPVT(data);
    if (type_ == GNSS_TYPE_UBLOX) return ublox_.pollNewPVT(data);
    if (type_ == GNSS_TYPE_SDR) return sdr_.pollNewPVT(data);   // reads the UART; its fix on TR_GNSS_AUTO_RAW
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    if (type_ == GNSS_TYPE_PX1105R) return px_.pollNewPVT(data);
#endif
    return false;
}

bool TR_GNSSAuto::pollNewSat(GNSSSatData& out)
{
    return type_ == GNSS_TYPE_UBLOX && ublox_.pollNewSat(out);
}

bool TR_GNSSAuto::pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us)
{
    if (type_ == GNSS_TYPE_SDR) return sdr_.pollRawFrame(out, cap, len, time_us);
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    if (type_ == GNSS_TYPE_PX1105R) return px_.pollRawFrame(out, cap, len, time_us);
#endif
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.pollRawFrame(out, cap, len, time_us);
}

bool TR_GNSSAuto::takeClockLine(GnssClockData& out)
{
    if (type_ == GNSS_TYPE_SDR) return sdr_.takeClockLine(out);
#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
    if (type_ == GNSS_TYPE_PX1105R) return px_.takeClockLine(out);   // its 0xE5 arrival line
#endif
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.takeClockLine(out);
}

bool TR_GNSSAuto::sendAidFrame(const uint8_t* frame, size_t len)
{
    return type_ == GNSS_TYPE_SDR && sdr_.sendAidFrame(frame, len);
}

bool TR_GNSSAuto::geoidSeparationM(float& sep_m) const
{
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.geoidSeparationM(sep_m);
}

void TR_GNSSAuto::requestNavResend()
{
    if (type_ == GNSS_TYPE_MOSAIC) mosaic_.requestNavResend();
}

#if defined(TR_GNSS_AUTO_RAW) && TR_GNSS_AUTO_RAW
// ── The raw receivers' raw outputs (TR_GNSS_AUTO_RAW) ───────────────────

bool TR_GNSSAuto::beginHintedRaw(uint8_t hint, uint8_t rx, uint8_t tx, int8_t reset_n_pin, int8_t safeboot_n_pin)
{
    // Each driver's own restored-flight start: it returns at once and its
    // poll task brings the receiver up when it answers (the board's only
    // listens anyway). A u-blox or nothing known: false, the usual path.
    bool ok = false;
    if (hint == GNSS_TYPE_MOSAIC)
    {
        mosaic_.setBeginWait(false);
        ok = mosaic_.begin(mosaic_rate_hz_, rx, tx, reset_n_pin, safeboot_n_pin);
    }
    else if (hint == GNSS_TYPE_PX1105R)
    {
        px_.setBeginWait(false);
        ok = px_.begin(kPx1105rRateHz, rx, tx, reset_n_pin, safeboot_n_pin);
    }
    else if (hint == GNSS_TYPE_SDR)
    {
        ok = sdr_.begin(kSdrPollRateHz, rx, tx, reset_n_pin, safeboot_n_pin);
    }
    else
    {
        return false;
    }
    type_ = ok ? hint : GNSS_TYPE_NONE;
    ESP_LOGW(TAG, "GNSS auto: a restored flight: no probes, the %s it flew with (%s)", gnssTypeName(hint),
             ok ? "brought up by its poll task when it answers" : "its driver did not start");
    return true;
}

bool TR_GNSSAuto::rawReceiver() const
{
    return type_ == GNSS_TYPE_MOSAIC || type_ == GNSS_TYPE_SDR || type_ == GNSS_TYPE_PX1105R;
}

bool TR_GNSSAuto::enableRawOutputs()
{
    // Before begin(), as each image calls it: all three made ready (each
    // allocates its own block), so whichever begin() finds hands its raw
    // measurements over; the two not found never run.
    const bool m = mosaic_.enableRawOutputs();
    const bool s = sdr_.enableRawOutputs();
    const bool x = px_.enableRawOutputs();
    return m && s && x;
}

bool TR_GNSSAuto::takeRawEpoch(gnss_raw::TimedRawEpoch& out)
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.takeRawEpoch(out);
    if (type_ == GNSS_TYPE_SDR) return sdr_.takeRawEpoch(out);
    if (type_ == GNSS_TYPE_PX1105R) return px_.takeRawEpoch(out);
    return false;
}

bool TR_GNSSAuto::takeEphemeris(uint8_t* out, size_t cap, size_t& len)
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.takeEphemeris(out, cap, len);
    if (type_ == GNSS_TYPE_SDR) return sdr_.takeEphemeris(out, cap, len);
    return false;
}

bool TR_GNSSAuto::takeEphemeris(gps_eph_t& out)
{
    return type_ == GNSS_TYPE_PX1105R && px_.takeEphemeris(out);
}

bool TR_GNSSAuto::takeFix(gnss_raw::TimedFix& out)
{
    return type_ == GNSS_TYPE_PX1105R && px_.takeFix(out);
}

bool TR_GNSSAuto::broadcastIono(int8_t raw[8], uint16_t& gps_day) const
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.broadcastIono(raw, gps_day);
    if (type_ == GNSS_TYPE_SDR) return sdr_.broadcastIono(raw, gps_day);
    if (type_ == GNSS_TYPE_PX1105R) return px_.broadcastIono(raw, gps_day);
    return false;
}

void TR_GNSSAuto::setRawEpochPeriodMs(uint32_t period_ms)
{
    // Before begin() as in their images: both take it, the one found uses it
    mosaic_.setRawEpochPeriodMs(period_ms);
    px_.setRawEpochPeriodMs(period_ms);
}

void TR_GNSSAuto::setRawEpochConfig(uint32_t period_ms, const sdr_rtcm::SdrDopplerMeanConfig& mean)
{
    sdr_.setRawEpochConfig(period_ms, mean);
}

bool TR_GNSSAuto::setRawDopplerPairing(const mosaic::MosaicDopplerPairConfig& c)
{
    if (type_ == GNSS_TYPE_PX1105R) return px_.setRawDopplerPairing(c);
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.setRawDopplerPairing(c);
    return false;
}

void TR_GNSSAuto::rawStats(TR_GNSSReceiverMosaicSerial::RawStats& out) const { mosaic_.rawStats(out); }
void TR_GNSSAuto::rawStats(TR_GNSSReceiverSdrSerial::RawStats& out) const { sdr_.rawStats(out); }
void TR_GNSSAuto::rawStats(TR_GNSSReceiverPX1105RSerial::RawStats& out) const { px_.rawStats(out); }

bool TR_GNSSAuto::rawPairStats(TR_GNSSReceiverMosaicSerial::RawPairStats& out) const
{
    if (type_ == GNSS_TYPE_PX1105R)
    {
        // The PX1105R pairs by the mosaic's rule and counts the same things:
        // its counts in the mosaic's shape, which main.cpp's console reads
        TR_GNSSReceiverPX1105RSerial::RawPairStats px;
        if (!px_.rawPairStats(px)) return false;
        out.pair = px.pair;
        out.reads = px.reads;
        out.read_us_max = px.read_us_max;
        out.read_us_sum = px.read_us_sum;
        out.adapts = px.adapts;
        out.adapt_us_max = px.adapt_us_max;
        out.adapt_us_sum = px.adapt_us_sum;
        out.avg_us_max = px.avg_us_max;
        return true;
    }
    return mosaic_.rawPairStats(out);
}

bool TR_GNSSAuto::rawPairStats(TR_GNSSReceiverPX1105RSerial::RawPairStats& out) const
{
    return px_.rawPairStats(out);
}

bool TR_GNSSAuto::setRawMultiband(const mosaic::MosaicMultibandConfig& c, float windup_k)
{
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.setRawMultiband(c, windup_k);
}

void TR_GNSSAuto::noteRollRate(float p_rps)
{
    if (type_ == GNSS_TYPE_MOSAIC) mosaic_.noteRollRate(p_rps);
}

bool TR_GNSSAuto::rawMbStats(TR_GNSSReceiverMosaicSerial::RawMbStats& out) const
{
    return mosaic_.rawMbStats(out);
}

bool TR_GNSSAuto::heardBoard() const
{
    return type_ == GNSS_TYPE_SDR && sdr_.heardBoard();
}
#endif

#if defined(TR_GNSS_COCOM_DIAG) && TR_GNSS_COCOM_DIAG
void TR_GNSSAuto::logSatDiag()
{
    if (type_ == GNSS_TYPE_UBLOX) ublox_.logSatDiag();
}
#endif

#endif  // TR_GNSS_DRIVER_AUTO
