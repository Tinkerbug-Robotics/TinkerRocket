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

// The mosaic streams ~66 kB/s at 2 Mbaud, which fills the UART's 128-byte RX
// FIFO in 0.64 ms: less than a flash write keeps an interrupt that needs the
// cache waiting. sdkconfig.defaults.mosaic puts the UART interrupt in IRAM;
// a build dir made before this image had the overlay keeps its old sdkconfig.
#if !CONFIG_UART_ISR_IN_IRAM
#error "TR_GNSS_DRIVER_AUTO needs CONFIG_UART_ISR_IN_IRAM (sdkconfig.defaults.mosaic): use a fresh build dir, or delete its sdkconfig"
#endif

static const char* TAG = "GNSS";

namespace
{

constexpr size_t kRxBuf = 8192;   // ~120 ms of a set-up mosaic; the loop below reads every few ms
constexpr size_t kTxBuf = 256;

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
    : port_(uart_port), ublox_(uart_port), mosaic_(uart_port)
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
    // Each driver installs the UART its own way (the mosaic's with an event
    // queue and a 16 kB ring), and the mosaic's only re-rates one it finds
    // installed, so none is left for it.
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

    uint32_t ublox_baud = 0;
    bool ublox_swapped = false;
    ubloxLastLink(ublox_baud, ublox_swapped);
    gnss_detect::BringUp up;
    up.start(rememberedType(), ublox_baud, kept_power, ublox_swapped);
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
                    ESP_LOGW(TAG, "GNSS auto: no mosaic-G5 either (%lu ms)",
                             static_cast<unsigned long>(detector_.elapsedMs()));
                }
                else if (up.plan().n > 0)
                {
                    ESP_LOGW(TAG, "GNSS auto: no receiver answered the probes in %lu ms (%lu probes)%s",
                             static_cast<unsigned long>(detector_.elapsedMs()),
                             static_cast<unsigned long>(detector_.probesRun()),
                             up.plan().fallback == GNSS_TYPE_UBLOX
                                 ? "; the u-blox driver searches as it always has"
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
            default:
                break;
        }
    }

    type_ = up.active();
    if (type_ != GNSS_TYPE_NONE)
    {
        ESP_LOGI(TAG, "GNSS auto: flying the %s", gnssTypeName(type_));
    }
    return type_ != GNSS_TYPE_NONE;
}

// ── Calls after begin() ─────────────────────────────────────────────────

uint8_t TR_GNSSAuto::updateRateHz() const
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_rate_hz_ > 20 ? 20 : mosaic_rate_hz_;
    return ublox_rate_hz_;
}

uint8_t TR_GNSSAuto::otpState() const
{
    return (type_ == GNSS_TYPE_MOSAIC) ? mosaic_.otpState() : ublox_.otpState();
}

void TR_GNSSAuto::getGNSSData(GNSSData& data)
{
    if (type_ == GNSS_TYPE_MOSAIC)
        mosaic_.getGNSSData(data);
    else if (type_ == GNSS_TYPE_UBLOX)
        ublox_.getGNSSData(data);
    else
        data = GNSSData{};
}

bool TR_GNSSAuto::pollNewPVT(GNSSData& data)
{
    if (type_ == GNSS_TYPE_MOSAIC) return mosaic_.pollNewPVT(data);
    if (type_ == GNSS_TYPE_UBLOX) return ublox_.pollNewPVT(data);
    return false;
}

bool TR_GNSSAuto::pollNewSat(GNSSSatData& out)
{
    return type_ == GNSS_TYPE_UBLOX && ublox_.pollNewSat(out);
}

bool TR_GNSSAuto::pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us)
{
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.pollRawFrame(out, cap, len, time_us);
}

bool TR_GNSSAuto::takeClockLine(GnssClockData& out)
{
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.takeClockLine(out);
}

bool TR_GNSSAuto::geoidSeparationM(float& sep_m) const
{
    return type_ == GNSS_TYPE_MOSAIC && mosaic_.geoidSeparationM(sep_m);
}

void TR_GNSSAuto::requestNavResend()
{
    if (type_ == GNSS_TYPE_MOSAIC) mosaic_.requestNavResend();
}

#if defined(TR_GNSS_COCOM_DIAG) && TR_GNSS_COCOM_DIAG
void TR_GNSSAuto::logSatDiag()
{
    if (type_ == GNSS_TYPE_UBLOX) ublox_.logSatDiag();
}
#endif

#endif  // TR_GNSS_DRIVER_AUTO
