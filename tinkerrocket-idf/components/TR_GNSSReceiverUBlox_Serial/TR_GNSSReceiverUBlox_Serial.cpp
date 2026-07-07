#include "TR_GNSSReceiverUBlox_Serial.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <cstring>

static const char* TAG = "GNSS";

static constexpr size_t GNSS_UART_RX_BUF = 4096;
static constexpr size_t GNSS_UART_TX_BUF = 256;

// Constructor
TR_GNSSReceiverUBloxSerial::TR_GNSSReceiverUBloxSerial(uart_port_t uart_port)
    : _uartPort(uart_port) {}

// ── UART helpers ────────────────────────────────────────────────────────

void TR_GNSSReceiverUBloxSerial::uartBegin(uint32_t baud, uint8_t rx_pin, uint8_t tx_pin)
{
    // Tear down any previous driver on this port.
    uartEnd();

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = (int)baud;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(_uartPort, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(_uartPort, (int)tx_pin, (int)rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(_uartPort, GNSS_UART_RX_BUF,
                                        GNSS_UART_TX_BUF, 0, NULL, 0));
}

void TR_GNSSReceiverUBloxSerial::uartEnd()
{
    if (uart_is_driver_installed(_uartPort))
    {
        uart_driver_delete(_uartPort);
    }
}

size_t TR_GNSSReceiverUBloxSerial::uartAvailable()
{
    size_t buffered = 0;
    uart_get_buffered_data_len(_uartPort, &buffered);
    return buffered;
}

int TR_GNSSReceiverUBloxSerial::uartRead()
{
    uint8_t byte;
    int n = uart_read_bytes(_uartPort, &byte, 1, 0);
    return (n == 1) ? (int)byte : -1;
}

// ── begin() ─────────────────────────────────────────────────────────────

bool TR_GNSSReceiverUBloxSerial::begin(uint8_t update_rate_hz_in,
                                       uint8_t GNSS_RX,
                                       uint8_t GNSS_TX,
                                       int8_t reset_n_pin,
                                       int8_t safeboot_n_pin)
{
    update_rate_hz = update_rate_hz_in;

    ESP_LOGI(TAG, "Starting SAM-M10Q configuration...");

    const uint32_t preferred_baud = 460800U;
    const uint32_t bootstrap_baud = preferred_baud;
    const uint32_t probe_bauds[] = {9600U, 230400U, 115200U, 460800U, 38400U, 57600U};
    uint32_t connected_baud = 0U;
    uint8_t active_rx = GNSS_RX;
    uint8_t active_tx = GNSS_TX;

    if (safeboot_n_pin >= 0)
    {
        pinMode((uint8_t)safeboot_n_pin, OUTPUT);
        digitalWrite((uint8_t)safeboot_n_pin, HIGH);
        ESP_LOGI(TAG, "SAFEBOOT_N forced HIGH on pin %d", safeboot_n_pin);
        delay(10);
    }

    auto pulseReset = [&]()
    {
        if (reset_n_pin < 0)
        {
            return;
        }

        pinMode((uint8_t)reset_n_pin, OUTPUT);
        digitalWrite((uint8_t)reset_n_pin, HIGH);
        delay(2);
        digitalWrite((uint8_t)reset_n_pin, LOW);
        delay(20);
        digitalWrite((uint8_t)reset_n_pin, HIGH);
        delay(250);
        ESP_LOGI(TAG, "Pulsed RESET_N on pin %d", reset_n_pin);
    };

    auto hasSerialActivity = [&](uint8_t rx_pin, uint8_t tx_pin, uint32_t baud,
                                 uint32_t window_ms = 400U) -> bool
    {
        uartBegin(baud, rx_pin, tx_pin);

        const uint32_t start_ms = millis();
        while ((millis() - start_ms) < 250U)
        {
            // Drain any stale bytes
            uint8_t tmp;
            while (uart_read_bytes(_uartPort, &tmp, 1, 0) > 0) {}
            delay(1);
        }

        const uint32_t activity_window_start = millis();
        while ((millis() - activity_window_start) < window_ms)
        {
            if (uartAvailable() > 0)
            {
                ESP_LOGI(TAG, "Serial activity detected on RX=%d TX=%d at %lu baud",
                         rx_pin, tx_pin, (unsigned long)baud);
                return true;
            }
            delay(1);
        }
        return false;
    };

    auto scanAndConnectPins = [&](uint8_t rx_pin, uint8_t tx_pin, uint32_t &found_baud) -> bool
    {
        const size_t n = sizeof(probe_bauds) / sizeof(probe_bauds[0]);
        for (size_t i = 0; i < n; i++)
        {
            const uint32_t baud = probe_bauds[i];
            ESP_LOGI(TAG, "Trying %lu baud (RX=%d, TX=%d)",
                     (unsigned long)baud, rx_pin, tx_pin);

            uartBegin(baud, rx_pin, tx_pin);
            delay(100);

            if (gnss.begin(_uartPort, 1500) == true)
            {
                found_baud = baud;
                active_rx = rx_pin;
                active_tx = tx_pin;
                return true;
            }

            // If we see bytes but cannot establish full UBX handshake,
            // use assumeSuccess path and attempt to recover configuration.
            if (hasSerialActivity(rx_pin, tx_pin, baud) && gnss.begin(_uartPort, 1500, true))
            {
                found_baud = baud;
                active_rx = rx_pin;
                active_tx = tx_pin;
                ESP_LOGI(TAG, "Connected using serial signs-of-life");
                return true;
            }
        }
        return false;
    };

    auto scanAndConnect = [&](uint32_t &found_baud) -> bool
    {
        if (scanAndConnectPins(GNSS_RX, GNSS_TX, found_baud))
        {
            return true;
        }

        if ((GNSS_RX != GNSS_TX) && scanAndConnectPins(GNSS_TX, GNSS_RX, found_baud))
        {
            ESP_LOGW(TAG, "Detected swapped RX/TX wiring; using swapped pin assignment");
            return true;
        }
        return false;
    };

    // Bootstrap from preferred UART rate first (warm-boot fast path: module
    // already configured at preferred_baud from a previous run).
    ESP_LOGI(TAG, "Bootstrap try %lu baud", (unsigned long)bootstrap_baud);
    uartBegin(bootstrap_baud, GNSS_RX, GNSS_TX);
    delay(80);
    if (gnss.begin(_uartPort, 800))
    {
        connected_baud = preferred_baud;
        active_rx = GNSS_RX;
        active_tx = GNSS_TX;
    }

    // Warm-boot fast path for swapped-wiring boards: the module persists its
    // configured baud (preferred_baud) across resets, so a swapped rev comes
    // up already at preferred_baud on the swapped pins. Try that before the
    // 9600 orientation probe + full sweep — otherwise the sweep crawls
    // 9600→…→preferred (~30 s) even though the orientation was found quickly.
    if (connected_baud == 0U && GNSS_RX != GNSS_TX)
    {
        ESP_LOGI(TAG, "Bootstrap try %lu baud on swapped RX/TX", (unsigned long)bootstrap_baud);
        uartBegin(bootstrap_baud, GNSS_TX, GNSS_RX);
        delay(80);
        if (gnss.begin(_uartPort, 800))
        {
            connected_baud = preferred_baud;
            active_rx = GNSS_TX;
            active_tx = GNSS_RX;
            ESP_LOGW(TAG, "Bootstrap: connected at %lu baud on swapped RX/TX", (unsigned long)preferred_baud);
        }
    }

    // Fast cold-boot orientation probe: u-blox modules power up at 9600 baud
    // (factory default). A ~650 ms listen at 9600 on each orientation finds
    // which way the RX/TX is wired and short-circuits the slow ~30 s
    // baud-cycle scan on the wrong orientation. Works for both schematic-
    // labeled boards (default orientation) and rev's where RX/TX are swapped.
    if (connected_baud == 0U)
    {
        uint8_t probe_rx = GNSS_RX;
        uint8_t probe_tx = GNSS_TX;
        bool    detected = false;
        bool    swapped  = false;

        // u-blox modules power up at 9600 baud / 1 Hz output, so the listen
        // window must exceed one full output period (~1 s) to catch a burst
        // regardless of phase. A 400 ms window missed it intermittently and
        // dropped boot into the ~30 s full baud scan (bench 2026-05-29). Probe
        // the schematic orientation first, then the swapped wiring — both rev's
        // now detect reliably in one or two ~1.2 s windows instead of ~35 s.
        const uint32_t kOrientationProbeMs = 1200U;
        if (hasSerialActivity(GNSS_RX, GNSS_TX, 9600U, kOrientationProbeMs))
        {
            detected = true;
        }
        else if ((GNSS_RX != GNSS_TX) && hasSerialActivity(GNSS_TX, GNSS_RX, 9600U, kOrientationProbeMs))
        {
            detected = true;
            swapped  = true;
            probe_rx = GNSS_TX;
            probe_tx = GNSS_RX;
        }

        if (detected)
        {
            if (swapped)
            {
                ESP_LOGW(TAG, "Quick probe: activity at 9600 on swapped RX/TX "
                              "(RX=%d TX=%d)", probe_rx, probe_tx);
            }
            uint32_t probe_found = 0U;
            if (scanAndConnectPins(probe_rx, probe_tx, probe_found))
            {
                connected_baud = probe_found;
                // active_rx/tx set inside scanAndConnectPins
            }
        }
    }

    // Fall-back full-scan loop. Stays as a safety net for the case where
    // the quick orientation probe fails (e.g. module powered on at a
    // non-default baud and the activity probe at 9600 saw nothing, or the
    // initial UBX handshake never completes despite serial activity).
    uint8_t scan_attempt = 0;
    while ((connected_baud == 0U) && !scanAndConnect(connected_baud))
    {
        scan_attempt++;
        ESP_LOGW(TAG, "No response on known bauds, retrying...");
        if ((scan_attempt % 2U) == 0U)
        {
            pulseReset();
        }
        delay(700);
    }

    ESP_LOGI(TAG, "Connected at %lu baud", (unsigned long)connected_baud);
    ESP_LOGI(TAG, "Active UART pins RX=%d TX=%d", active_rx, active_tx);

    // Standardize to preferred baud for runtime.
    // We require preferred_baud for runtime consistency and throughput.
    if (connected_baud != preferred_baud)
    {
        bool switched = false;
        for (uint8_t attempt = 0; attempt < 6; attempt++)
        {
            bool baud_change_requested = false;
            for (uint8_t n = 0; n < 3; n++)
            {
                if (gnss.setSerialRate(preferred_baud))
                {
                    baud_change_requested = true;
                    break;
                }
                delay(60);
            }

            if (!baud_change_requested)
            {
                ESP_LOGW(TAG, "No ACK on baud switch request; forcing probe on preferred baud");
            }

            uartBegin(preferred_baud, active_rx, active_tx);
            delay(150);

            if (gnss.begin(_uartPort, 1500))
            {
                connected_baud = preferred_baud;
                switched = true;
                break;
            }

            // Recover command path on old baud for next attempt.
            uartBegin(connected_baud, active_rx, active_tx);
            delay(120);
            (void)gnss.begin(_uartPort, 1500, true);
            delay(80);
        }

        if (!switched)
        {
            ESP_LOGW(TAG, "Failed to force preferred baud; applying factory default and rescanning...");
            (void)gnss.factoryDefault(5000);
            delay(1500);

            while (!scanAndConnect(connected_baud))
            {
                delay(500);
            }

            if (connected_baud != preferred_baud)
            {
                (void)gnss.setSerialRate(preferred_baud);
                uartBegin(preferred_baud, active_rx, active_tx);
                delay(200);
                if (gnss.begin(_uartPort, 1500))
                {
                    connected_baud = preferred_baud;
                }
                else
                {
                    // Hard requirement: runtime must be preferred_baud.
                    ESP_LOGE(TAG, "Device not at preferred baud after recovery");
                    return false;
                }
            }
        }

        ESP_LOGI(TAG, "Running at %lu baud", (unsigned long)preferred_baud);
    }

    // Runtime always enforced at preferred_baud; probe list includes fallback
    // rates for modules that are not yet configured.

    ESP_LOGI(TAG, "Serial connected, verified runtime baud = %lu", (unsigned long)connected_baud);

    bool module_is_m10 = false;
    if (gnss.getModuleInfo())
    {
        ESP_LOGI(TAG, "Module: %s", gnss.getModuleName());
        ESP_LOGI(TAG, "Firmware type: %s", gnss.getFirmwareType());
        ESP_LOGI(TAG, "Firmware version: %d.%d",
                 gnss.getFirmwareVersionHigh(), gnss.getFirmwareVersionLow());
        ESP_LOGI(TAG, "Protocol version: %d.%d",
                 gnss.getProtocolVersionHigh(), gnss.getProtocolVersionLow());
        module_is_m10 = (strstr(gnss.getModuleName(), "M10") != nullptr);
    }

    // ── High performance navigation update rate (§2.1.5, UBX-22020019) ──
    // The M10 defaults to a low CPU clock whose nav-rate ceiling (~10 Hz with
    // 4 concurrent constellations) is below our configured GNSS_UPDATE_RATE.
    // The high-clock configuration lives in OTP memory: programmed once,
    // applied automatically at every startup, PERMANENT.  Verify it at every
    // boot (new/replacement modules arrive unprogrammed) and program it —
    // with the manual's exact byte strings — when absent.  Gated on a
    // positively identified M10 so we can never burn OTP on a different part.
    if (module_is_m10 && !ensureHighPerformanceClock())
    {
        // OTP just programmed: it only applies at startup, so hardware-reset
        // the receiver and redo the whole connect+configure once.
        if (!otp_reset_done_)
        {
            otp_reset_done_ = true;
            ESP_LOGW(TAG, "Resetting receiver to apply OTP high-clock config");
            pulseReset();
            delay(500);
            return begin(update_rate_hz_in, GNSS_RX, GNSS_TX,
                         reset_n_pin, safeboot_n_pin);
        }
        ESP_LOGE(TAG, "High-performance clock still not verified after OTP "
                      "write + reset — continuing at default clock (reduced "
                      "nav-rate ceiling)");
    }

    auto configureReceiver = [&]() -> bool
    {
        bool ok = false;
        uint8_t i = 0;

        // Accept both UBX and NMEA on input (for bring-up compatibility).
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setUART1Input((uint8_t)(COM_TYPE_UBX | COM_TYPE_NMEA))) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to set UART1 input protocol mask");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "UART1 input protocol mask set");

        // UBX-only output: NMEA sentences (GGA, RMC, GSV x 4 constellations)
        // add ~1-2 KB/epoch of serial data that the SparkFun library must parse
        // byte-by-byte, blocking the sensor polling task for ~10 ms per GNSS
        // poll and causing ISM6/BMP/MMC data gaps.  We only need UBX autoPVT.
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setUART1Output(COM_TYPE_UBX)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to set UART1 output protocol mask");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "UART1 output protocol mask set (UBX only)");

        // All four constellations for maximum satellite visibility.
        // The SAM-M10Q caps at 10 Hz with 4 concurrent constellations,
        // but the extra sats are more valuable for rocket flight than
        // higher nav rate.  We request slightly above 10 Hz (via config)
        // so the receiver runs at its true ceiling even when the actual
        // rate drops with satellite count.
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to enable GPS constellation");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Enabled GPS");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to enable Galileo constellation");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Enabled Galileo");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to enable Glonass constellation");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Enabled Glonass");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to enable Beidou constellation");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Enabled Beidou");

        // Power mode: ensure Full Power (continuous / high performance).
        // CFG-PM-OPERATEMODE: 0 = Full Power, 1 = PSMOO, 2 = PSMCT
        {
            uint8_t cur_mode = 0xFF;
            if (gnss.getVal8(UBLOX_CFG_PM_OPERATEMODE, &cur_mode))
            {
                ESP_LOGI(TAG, "Current power mode = %u", cur_mode);
                if (cur_mode != 0)
                {
                    ESP_LOGI(TAG, "Switching to Full Power (high performance) mode");
                    ok = false;
                    for (i = 0; i < 8; i++)
                    {
                        if (gnss.setVal8(UBLOX_CFG_PM_OPERATEMODE, 0, VAL_LAYER_RAM_BBR)) { ok = true; break; }
                        ESP_LOGW(TAG, "Failed to set Full Power mode");
                        delay(150);
                    }
                    if (!ok) return false;
                    ESP_LOGI(TAG, "Full Power mode set");
                }
                else
                {
                    ESP_LOGI(TAG, "Already in Full Power mode");
                }
            }
            else
            {
                // Cannot read -> force-write Full Power as a safe default
                ESP_LOGW(TAG, "Cannot read power mode, forcing Full Power");
                (void)gnss.setVal8(UBLOX_CFG_PM_OPERATEMODE, 0, VAL_LAYER_RAM_BBR);
            }
        }

        // Dynamic model: Airborne <4g (DYN_MODEL_AIRBORNE4g = 8).
        // NOTE (#174): value 6 is Airborne <1g, not <4g — the receiver then
        // assumes <1g dynamics and drops the fix the instant the motor lights
        // (>1g), not reacquiring until the rocket slows on descent, which
        // starved the EKF of GNSS through the whole boost+coast.
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setVal8(UBLOX_CFG_NAVSPG_DYNMODEL, DYN_MODEL_AIRBORNE4g)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to set dynamic model");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Dynamic model set to Airborne <4g");

        // Read it back (#242): on the 6/14 flights the GPS solution was
        // corrupted through boost (position dive + vel_u noise) at fix=3 with
        // good reported accuracy — confirm the model actually took, since a
        // silently-rejected set would explain the boost-phase de-weighting.
        {
            const uint8_t actual_dynmodel = gnss.getDynamicModel();
            if (actual_dynmodel == DYN_MODEL_AIRBORNE4g)
                ESP_LOGI(TAG, "Dynamic model readback OK: Airborne <4g (%u)",
                         (unsigned)actual_dynmodel);
            else
                ESP_LOGW(TAG, "Dynamic model readback MISMATCH (#242): set %u "
                              "(Airborne <4g) but reads %u",
                         (unsigned)DYN_MODEL_AIRBORNE4g, (unsigned)actual_dynmodel);
        }

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setNavigationFrequency(update_rate_hz)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to set navigation update rate");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Navigation update rate set");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setAutoPVT(true)) { ok = true; break; }
            ESP_LOGW(TAG, "Failed to enable auto PVT");
            delay(150);
        }
        if (!ok) return false;
        ESP_LOGI(TAG, "Auto PVT enabled");

        // Save settings to BBR/flash where available.
        (void)gnss.saveConfiguration();
        return true;
    };

    // New modules can be in odd config states. If configuration repeatedly fails,
    // perform a factory default and try one more full configuration pass.
    if (!configureReceiver())
    {
        ESP_LOGW(TAG, "Config failed, applying factory default and retrying...");
        (void)gnss.factoryDefault(5000);
        delay(1500);

        while (!scanAndConnect(connected_baud))
        {
            delay(500);
        }

        if (connected_baud != preferred_baud)
        {
            (void)gnss.setSerialRate(preferred_baud);
            uartBegin(preferred_baud, active_rx, active_tx);
            delay(150);
            (void)gnss.begin(_uartPort, 1500);
        }

        if (!configureReceiver())
        {
            ESP_LOGE(TAG, "Configuration failed after factory default");
            return false;
        }
    }

    ESP_LOGI(TAG, "Configuration complete.");
    delay(100);

    return true;
}

// SAM-M10Q high performance navigation update rate (integration manual
// UBX-22020019 §2.1.5).  The high-CPU-clock configuration lives in OTP
// (one-time programmable) memory: written once, applied automatically at
// every startup, permanent and irreversible.  Verification and programming
// below use the manual's byte-exact sequences (keys/values are undocumented
// M10 internals — do NOT invent values).
//
// Returns true when the high-performance clock is verified present (or was
// already programmed this boot and re-verified).  Returns false when the OTP
// string was just written — the caller must hardware-reset the receiver and
// reconnect, since the OTP config only applies at startup.
bool TR_GNSSReceiverUBloxSerial::ensureHighPerformanceClock()
{
    // §2.1.5 step 5: VALGET (layer 0x04) of the OTP clock keys.  High CPU
    // clock reads 0x0B71B000 at keys 0x40A40001/03/05 and 0x05B8D800 at
    // 0x40A4000A; two keys (one of each expected value) are enough signal.
    constexpr uint32_t KEY_CLK_A = 0x40A40003;
    constexpr uint32_t KEY_CLK_B = 0x40A4000A;
    constexpr uint32_t HI_CLK_A  = 0x0B71B000;
    constexpr uint32_t HI_CLK_B  = 0x05B8D800;
    constexpr uint8_t  OTP_LAYER = 0x04;  // layer byte from the manual's poll

    uint32_t a = 0, b = 0;
    bool read_ok = false;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (gnss.getVal32(KEY_CLK_A, &a, OTP_LAYER, 1100) &&
            gnss.getVal32(KEY_CLK_B, &b, OTP_LAYER, 1100))
        {
            read_ok = true;
            break;
        }
        delay(150);
    }

    if (read_ok && a == HI_CLK_A && b == HI_CLK_B)
    {
        ESP_LOGI(TAG, "High-performance clock OTP config verified "
                      "(0x%08lX / 0x%08lX)",
                 (unsigned long)a, (unsigned long)b);
        return true;
    }

    if (read_ok)
    {
        ESP_LOGW(TAG, "OTP clock keys read 0x%08lX / 0x%08lX — NOT the "
                      "high-performance configuration",
                 (unsigned long)a, (unsigned long)b);
    }
    else
    {
        // An unprogrammed module NACKs the OTP-layer poll, so a persistent
        // read failure is itself the "not programmed" signal.
        ESP_LOGW(TAG, "OTP clock keys unreadable — treating as unprogrammed");
    }

    if (otp_program_attempted_)
    {
        return false;  // already wrote this boot; caller logs the failure
    }
    otp_program_attempted_ = true;

    // §2.1.5 Table 3 configuration string, byte-exact (two UBX-CFG 0x06/0x41
    // frames; sendCommand recomputes the checksums).  PERMANENT.
    ESP_LOGW(TAG, "Programming high-performance clock into OTP "
                  "(one-time, permanent, ~18 B of OTP)");

    static uint8_t otp1[16] = {0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E, 0x79, 0xBF,
                               0x28, 0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF};
    static uint8_t otp2[28] = {0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34, 0xF9, 0x12,
                               0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40,
                               0x00, 0xB0, 0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40,
                               0x00, 0xD8, 0xB8, 0x05};

    ubxPacket pkt = {0x06, 0x41, 0, 0, 0, nullptr, 0, 0,
                     SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
                     SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

    pkt.len = sizeof(otp1);
    pkt.payload = otp1;
    const sfe_ublox_status_e s1 = gnss.sendCommand(&pkt, 1100);
    pkt.len = sizeof(otp2);
    pkt.payload = otp2;
    const sfe_ublox_status_e s2 = gnss.sendCommand(&pkt, 1100);

    const bool acked =
        (s1 == SFE_UBLOX_STATUS_DATA_SENT || s1 == SFE_UBLOX_STATUS_DATA_RECEIVED) &&
        (s2 == SFE_UBLOX_STATUS_DATA_SENT || s2 == SFE_UBLOX_STATUS_DATA_RECEIVED);
    if (acked)
    {
        ESP_LOGW(TAG, "OTP high-clock config written and ACKed");
    }
    else
    {
        ESP_LOGE(TAG, "OTP write not ACKed (s1=%d s2=%d)", (int)s1, (int)s2);
    }
    return false;  // reset + re-verify either way
}

bool TR_GNSSReceiverUBloxSerial::pollNewPVT(GNSSData &gnss_data)
{
    // Parse any pending serial bytes (non-blocking).
    gnss.checkUblox();

    // getPVT() with autoPVT enabled is non-blocking: returns true only
    // when a new NAV-PVT message has been fully received and parsed.
    if (!gnss.getPVT(0))
        return false;

    // New PVT available -- read all fields (no implicit serial parsing
    // needed since getPVT already updated the cache).
    getGNSSData(gnss_data);
    return true;
}

void TR_GNSSReceiverUBloxSerial::getGNSSData(GNSSData &gnss_data)
{
    gnss_data.time_us = micros();
    gnss_data.year = gnss.getYear();
    gnss_data.month = gnss.getMonth();
    gnss_data.day = gnss.getDay();
    gnss_data.hour = gnss.getHour();
    gnss_data.minute = gnss.getMinute();
    gnss_data.second = gnss.getSecond();
    gnss_data.milli_second = gnss.getMillisecond();
    // 0: No Fix, 1: Dead Reckoning, 2: 2D Fix, 3: 3D Fix,
    // 4:GNSS + Dead Reckoning, 5: Time Only
    gnss_data.fix_mode = gnss.getFixType();
    gnss_data.num_sats = gnss.getSIV();

    // SparkFun u-blox returns PDOP as scale 0.01. Convert to x10 for packed type.
    const uint16_t pdop_x100 = gnss.getPDOP();
    uint16_t pdop_x10_u16 = (uint16_t)((pdop_x100 + 5U) / 10U);
    if (pdop_x10_u16 > 255U) pdop_x10_u16 = 255U;
    gnss_data.pdop_x10 = (uint8_t)pdop_x10_u16;

    // Accuracy estimates are reported in mm. Convert to whole meters.
    const uint32_t h_acc_mm = gnss.getHorizontalAccEst();
    const uint32_t v_acc_mm = gnss.getVerticalAccEst();
    uint32_t h_acc_m_u32 = (h_acc_mm + 500U) / 1000U;
    uint32_t v_acc_m_u32 = (v_acc_mm + 500U) / 1000U;
    if (h_acc_m_u32 > 255U) h_acc_m_u32 = 255U;
    if (v_acc_m_u32 > 255U) v_acc_m_u32 = 255U;
    gnss_data.h_acc_m = (uint8_t)h_acc_m_u32;
    gnss_data.v_acc_m = (uint8_t)v_acc_m_u32;

    // Velocity (ENU, mm/s)
    gnss_data.vel_e_mmps = gnss.getNedEastVel();
    gnss_data.vel_n_mmps = gnss.getNedNorthVel();
    gnss_data.vel_u_mmps = -gnss.getNedDownVel();

    // Latitude and Longitude (deg*1e7)
    gnss_data.lat_e7 = gnss.getLatitude();
    gnss_data.lon_e7 = gnss.getLongitude();

    // Altitude relative to mean sea level (mm)
    gnss_data.alt_mm = gnss.getAltitudeMSL();

    // --- Staleness detection ---
    // If the GNSS second+millisecond are unchanged across consecutive reads,
    // the SparkFun library is returning cached data (serial buffer overflow
    // caused the UBX parser to lose frame sync).  After STALE_THRESHOLD
    // consecutive unchanged readings, zero fix_mode so downstream consumers
    // know the position is unreliable.
    if (gnss_data.second == prev_second &&
        gnss_data.milli_second == prev_milli_second)
    {
        if (stale_count < UINT16_MAX) stale_count++;

        if (stale_count == STALE_THRESHOLD)
        {
            ESP_LOGW(TAG, "Data stale — no new PVT for 5 consecutive reads, zeroing fix_mode");
        }
    }
    else
    {
        if (stale_count >= STALE_THRESHOLD)
        {
            ESP_LOGI(TAG, "Data resumed after %u stale reads",
                          (unsigned)stale_count);
        }
        stale_count = 0;
    }

    prev_second = gnss_data.second;
    prev_milli_second = gnss_data.milli_second;

    if (stale_count >= STALE_THRESHOLD)
    {
        gnss_data.fix_mode = 0;  // signal "no fix" to consumers
    }
}
