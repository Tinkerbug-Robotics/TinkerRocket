#include "TR_GNSSReceiverUBlox_Serial.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <cstring>
#include <TR_NVS.h>  // Preferences — once-ever OTP write guard
#include "gnss_framing.h"

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
    // Order matters far more than it looks — see the deadline note below.
    // 38400 FIRST because this is an M10-generation module: the SAM-M10Q's
    // factory-default UART1 rate is 38400 8N1 (9600 is only used in safeboot).
    // The old list led with 9600, which is the M8-era default, and buried 38400
    // in 5th place — where the 35 s deadline expired before ever reaching it.
    // A cold module therefore NEVER got probed at the one rate it was actually
    // talking at (bench 2026-08-03: bytes seen on the wire, zero handshakes).
    // 460800 is our own configured rate, tried by the bootstrap above already.
    const uint32_t probe_bauds[] = {38400U, 9600U, 230400U, 115200U, 460800U, 57600U};
    uint32_t connected_baud = 0U;
    uint8_t active_rx = GNSS_RX;
    uint8_t active_tx = GNSS_TX;

    // Bring-up deadline (#557).  A dead or deaf-UART module used to hang begin()
    // forever in the retry loops below — delay() feeds the WDT, so the FC sat
    // silently on the pad with no telemetry and no flight.
    //
    // THE OLD BUDGET WAS SIZED AGAINST A TIMING MODEL THAT IS ~6x OPTIMISTIC.
    // It assumed "6 bauds x 2 orientations, each with a ~1.5 s begin() timeout
    // is ~27 s".  A baud only costs ~1.5 s when hasSerialActivity() sees
    // NOTHING.  That check is a bare uartAvailable() > 0 with no framing
    // validation, so a real stream sampled at the WRONG baud still delivers
    // bytes and still trips it — and then the expensive branch runs a 4500 ms
    // begin(), a 250 ms drain, an activity window, and a second 4500 ms
    // assumeSuccess begin().  Measured on the bench 2026-08-03: "Trying 9600"
    // at t=7530 ms to "Trying 230400" at t=17006 ms = 9.5 s for ONE baud.
    // Six of those is ~57 s, not 27 s, so 35 s never covered a full sweep — it
    // died around the third entry.  With 38400 moved to the front a healthy
    // cold module now connects on the first try, but keep the budget honest:
    // 60 s covers a real worst-case sweep instead of silently truncating it.
    // On expiry begin() returns
    // false and the collector continues in a GNSS-absent degraded mode.  The
    // loops still run one full attempt first (the deadline is only checked before
    // a *retry*), so a live-but-slow module is never cut off mid-sweep.  Compared
    // wrap-safe; boot-time millis() never wraps, but keep the idiom consistent.
    const uint32_t kBeginTimeoutMs   = 60000U;
    const uint32_t begin_deadline_ms = millis() + kBeginTimeoutMs;
    auto beginExpired = [&]() -> bool {
        return (int32_t)(millis() - begin_deadline_ms) >= 0;
    };

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

    // Does the stream at this baud actually FRAME as GNSS traffic?
    //
    // This used to be a bare `uartAvailable() > 0`, which is true for ANY byte
    // — and a real 9600 NMEA stream sampled at 38400 still delivers bytes.
    // Every wrong baud therefore looked alive and ran the expensive recovery
    // branch (1.5 s begin + 250 ms drain + 1.5 s assumeSuccess begin).  The
    // bring-up-deadline comment above has described this since 2026-08-03;
    // moving 38400 to the front of probe_bauds treated the symptom for modules
    // that really are at 38400, and made it worse for the common 9600 module.
    // Measured on the bench 2026-08-19: a healthy SAM-M10Q talking 9600 was
    // false-positived at 38400 and burned 9.4 s before the sweep got back to
    // 9600.
    //
    // Requiring real framing makes a wrong baud fail in ONE window instead:
    // either UBX sync (0xB5 0x62) or a complete NMEA sentence whose XOR
    // checksum validates.  A baud mismatch shreds byte boundaries, so a
    // spurious valid checksum is vanishingly unlikely, while a genuine stream
    // produces one inside a single output period.  A module that is alive but
    // emitting neither (mid-reconfiguration, binary-only at an odd rate) now
    // falls through to the full sweep, which is where it belongs — and the
    // 60 s deadline plus GNSS-absent degraded mode still backstop it.
    // The framing test itself lives in gnss_framing.h so the host tests can
    // reach it; see that header for why byte-presence alone was not enough.

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

        // One NMEA sentence is <= 82 bytes; 256 holds a couple plus slack, so a
        // sentence that starts mid-window still completes inside the buffer.
        uint8_t win[256];
        size_t  n = 0;
        const uint32_t activity_window_start = millis();
        while ((millis() - activity_window_start) < window_ms)
        {
            uint8_t b;
            while (n < sizeof(win) && uart_read_bytes(_uartPort, &b, 1, 0) > 0)
            {
                win[n++] = b;
            }
            if (tr::gnssFramingDetected(win, n))
            {
                ESP_LOGI(TAG, "GNSS framing detected on RX=%d TX=%d at %lu baud "
                              "(%u bytes)",
                         rx_pin, tx_pin, (unsigned long)baud, (unsigned)n);
                return true;
            }
            if (n >= sizeof(win)) break;   // buffer full of unframed noise
            delay(1);
        }
        if (n > 0)
        {
            ESP_LOGD(TAG, "%u bytes at %lu baud but no UBX/NMEA framing — "
                          "wrong baud, moving on",
                     (unsigned)n, (unsigned long)baud);
        }
        return false;
    };

    // `first_baud` (0 = none) is tried before the standard order.  Callers who
    // already have evidence for a rate — the 9600 orientation probe, or the
    // NVS last-good record — pass it here so the sweep does not throw that
    // knowledge away.  Before this, the orientation probe would confirm 9600
    // and then hand off to a sweep that starts at 38400 (bench 2026-08-19:
    // "activity at 9600" at t=6463 ms, actually connected at 9600 at
    // t=16055 ms — 9.4 s spent re-deriving what was already known).
    auto scanAndConnectPins = [&](uint8_t rx_pin, uint8_t tx_pin, uint32_t &found_baud,
                                  uint32_t first_baud = 0U) -> bool
    {
        const size_t n = sizeof(probe_bauds) / sizeof(probe_bauds[0]);
        for (size_t i = 0; i < n + 1; i++)
        {
            // #557: bail mid-sweep on the bring-up deadline.  A removed module
            // whose UART floats reads noise, so hasSerialActivity() trips and
            // each baud runs a double begin() (~5 s) — a full 12-baud sweep is
            // ~60 s.  Checking the deadline only *between* whole sweeps let a
            // dead/noisy module spin ~100 s (bench 2026-07-21); a per-baud check
            // bounds bring-up to roughly the deadline + one baud probe.
            if (beginExpired()) return false;
            // i == 0 is the hinted rate; the rest is the standard order with the
            // hint skipped so it is never probed twice.
            uint32_t baud;
            if (i == 0)
            {
                if (first_baud == 0U) continue;
                baud = first_baud;
            }
            else
            {
                baud = probe_bauds[i - 1];
                if (baud == first_baud) continue;
            }
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

    // Last-good link, remembered across boots.
    //
    // A given airframe wakes on the same rate and the same wiring every time —
    // this bench SAM-M10Q lands on 9600 default-orientation on every boot — yet
    // bring-up re-derived it from scratch each power-up.  Recording the rate
    // and orientation that actually worked turns the common case into one
    // ~1.6 s attempt instead of a bootstrap plus a sweep.
    //
    // Advisory ONLY.  A wrong or stale record (module swapped, board rewired,
    // receiver factory-reset back to 9600) costs one failed attempt and then
    // falls through to the untouched bootstrap + sweep, so it can slow a boot
    // slightly but can never prevent one.  That is why it is not gated on the
    // module id: a swap self-corrects on the first boot after it.
    struct LastLink { uint32_t baud; uint8_t rx; uint8_t tx; bool valid; };
    auto loadLastLink = [&]() -> LastLink
    {
        LastLink l{0U, 0U, 0U, false};
        Preferences prefs;
        if (!prefs.begin("gnsslink", true)) return l;   // read-only; absent on first boot
        l.baud = prefs.getUInt("baud", 0U);
        l.rx   = prefs.getUChar("rx", 0xFF);
        l.tx   = prefs.getUChar("tx", 0xFF);
        prefs.end();
        // Only honour a record that matches how this build is wired; a board
        // header change must not send us probing pins that are no longer GNSS.
        const bool pins_known = (l.rx == GNSS_RX && l.tx == GNSS_TX) ||
                                (l.rx == GNSS_TX && l.tx == GNSS_RX);
        l.valid = (l.baud != 0U) && pins_known;
        return l;
    };
    auto saveLastLink = [&](uint32_t baud, uint8_t rx, uint8_t tx)
    {
        Preferences prefs;
        if (!prefs.begin("gnsslink", false)) return;
        // Write only on change — NVS is flash, and this runs every boot.
        if (prefs.getUInt("baud", 0U) != baud ||
            prefs.getUChar("rx", 0xFF) != rx ||
            prefs.getUChar("tx", 0xFF) != tx)
        {
            prefs.putUInt("baud", baud);
            prefs.putUChar("rx", rx);
            prefs.putUChar("tx", tx);
            ESP_LOGI(TAG, "Remembered GNSS link: %lu baud RX=%d TX=%d",
                     (unsigned long)baud, rx, tx);
        }
        prefs.end();
    };

    // Fast path: retry exactly what worked last time, before the bootstrap.
    {
        const LastLink last = loadLastLink();
        if (last.valid)
        {
            ESP_LOGI(TAG, "Trying remembered GNSS link: %lu baud RX=%d TX=%d",
                     (unsigned long)last.baud, last.rx, last.tx);
            uartBegin(last.baud, last.rx, last.tx);
            delay(80);
            if (gnss.begin(_uartPort, 1500))
            {
                connected_baud = last.baud;
                active_rx      = last.rx;
                active_tx      = last.tx;
                ESP_LOGI(TAG, "Remembered GNSS link came up on the first try");
            }
            else
            {
                ESP_LOGI(TAG, "Remembered link did not answer — full bring-up");
            }
        }
    }

    // Bootstrap from preferred UART rate first (warm-boot fast path: module
    // already configured at preferred_baud from a previous run).
    // Skipped when the remembered link above already answered.
    if (connected_baud == 0U)
    {
        ESP_LOGI(TAG, "Bootstrap try %lu baud", (unsigned long)bootstrap_baud);
        uartBegin(bootstrap_baud, GNSS_RX, GNSS_TX);
        delay(80);
        if (gnss.begin(_uartPort, 800))
        {
            connected_baud = preferred_baud;
            active_rx = GNSS_RX;
            active_tx = GNSS_TX;
        }
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
            // The probe just proved framed traffic at 9600 on these pins, so
            // start there instead of at the head of probe_bauds.
            if (scanAndConnectPins(probe_rx, probe_tx, probe_found, 9600U))
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
        // #557: give up after the deadline rather than spinning forever on a
        // dead/deaf-UART module.  scanAndConnect() above already ran one full
        // sweep this iteration, so a live-but-slow module always gets at least
        // one complete attempt before we can bail here.
        if (beginExpired())
        {
            ESP_LOGE(TAG, "GNSS bring-up: no response after %lu ms (%u sweeps); "
                          "continuing without GNSS", (unsigned long)kBeginTimeoutMs,
                     scan_attempt);
            return false;
        }
        ESP_LOGW(TAG, "No response on known bauds, retrying...");
        if ((scan_attempt % 2U) == 0U)
        {
            pulseReset();
        }
        delay(700);
    }

    ESP_LOGI(TAG, "Connected at %lu baud", (unsigned long)connected_baud);
    ESP_LOGI(TAG, "Active UART pins RX=%d TX=%d", active_rx, active_tx);

    // Remember the rate the module was FOUND at, not the preferred rate it is
    // about to be standardized to below.  Those differ, and the found rate is
    // the one that predicts the next boot: this receiver comes up at the 9600
    // factory default every power cycle, because V_BCKP is unconnected on the
    // gnss-sam10m8-18mm-hv carrier (U1 pin 3, confirmed in the netlist), so it
    // keeps no backup domain and forgets its configured rate. Saving
    // preferred_baud here would hand the next boot a hint that is wrong on
    // exactly the hardware this is meant to speed up.
    saveLastLink(connected_baud, active_rx, active_tx);

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
                if (beginExpired())
                {
                    ESP_LOGE(TAG, "GNSS baud recovery timed out after %lu ms; "
                                  "continuing without GNSS", (unsigned long)kBeginTimeoutMs);
                    return false;
                }
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
            if (beginExpired())
            {
                ESP_LOGE(TAG, "GNSS config recovery timed out after %lu ms; "
                              "continuing without GNSS", (unsigned long)kBeginTimeoutMs);
                return false;
            }
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
// Auto-programming master switch.  When false the boot check is READ-ONLY:
// it reads and logs the module's OTP state (all four §2.1.5 keys
// individually) but never writes.  The OTP config budget is 69 bytes TOTAL
// (§2.3) and the high-perf config takes 18, so writes are a once-or-twice-
// per-module-lifetime resource; when enabled, writes only ever target a
// module whose OTP reads fully BLANK, at most once per module (NVS guard +
// the blocklist below).
static constexpr bool kOtpAutoProgram = true;

// Modules that must NEVER be auto-programmed, by UBX-SEC-UNIQID unique chip
// ID.  This travels with the FIRMWARE: the once-ever NVS guard lives on one
// main board, but GNSS daughter boards migrate between main boards, and a
// failed-attempt module READS BLANK — indistinguishable from factory-fresh
// by its own memory.  Any module whose write attempt fails must be added
// here so no other main board ever retries it.
//   B9A8090FB454 — bench module, 7/07: frame 2 of the OTP config string is
//   NAKed on content (per-frame AND contiguous-burst delivery, string
//   byte-identical across three u-blox manuals); most likely the interrupted
//   first attempt left its config region failing read-back.  Works normally
//   at the default clock.
static constexpr const char *kOtpNeverProgram[] = {
    "B9A8090FB454",
};

bool TR_GNSSReceiverUBloxSerial::ensureHighPerformanceClock()
{
    // §2.1.5 step 5 verification keys.  The decision is made from the
    // MODULE's own OTP contents (daughter boards move between main boards,
    // so host-side records can't be the primary source of truth):
    //   VERIFIED — all four keys read the high-clock values
    //   PARTIAL  — some keys readable or values mismatch: something is in
    //              the OTP already; NEVER auto-write over it (69-byte budget)
    //   BLANK    — all keys NACK (the unprogrammed signature): the only
    //              state eligible for programming
    struct OtpKey { uint32_t key; uint32_t expect; };
    static constexpr OtpKey kKeys[] = {
        {0x40A40001, 0x0B71B000},
        {0x40A40003, 0x0B71B000},
        {0x40A40005, 0x0B71B000},
        {0x40A4000A, 0x05B8D800},
    };
    constexpr uint8_t OTP_LAYER = 0x04;  // layer byte from the manual's poll

    uint8_t readable = 0, matching = 0;
    for (const auto &k : kKeys)
    {
        uint32_t v = 0;
        bool ok = false;
        for (uint8_t i = 0; i < 2 && !ok; i++)
        {
            ok = gnss.getVal32(k.key, &v, OTP_LAYER, 1100);
            if (!ok) delay(100);
        }
        if (ok)
        {
            readable++;
            if (v == k.expect) matching++;
            ESP_LOGI(TAG, "OTP key 0x%08lX = 0x%08lX (%s)",
                     (unsigned long)k.key, (unsigned long)v,
                     v == k.expect ? "expected" : "UNEXPECTED");
        }
        else
        {
            ESP_LOGI(TAG, "OTP key 0x%08lX: unreadable (NACK)", (unsigned long)k.key);
        }
    }

    if (matching == 4)
    {
        ESP_LOGI(TAG, "High-performance clock OTP config VERIFIED (4/4 keys)");
        return true;
    }
    if (readable > 0)
    {
        // Module memory says SOMETHING is programmed but not the expected
        // full config (partial write, or different content).  Never gamble
        // the remaining OTP budget on top of unknown content.
        ESP_LOGE(TAG, "OTP state PARTIAL/MISMATCHED (%u/4 readable, %u/4 "
                      "matching) — auto-programming refused; inspect with "
                      "u-center", readable, matching);
        return true;
    }
    ESP_LOGW(TAG, "OTP state BLANK (all keys NACK) — module unprogrammed");

    if (!kOtpAutoProgram)
    {
        ESP_LOGW(TAG, "OTP auto-programming DISABLED in this build (read-only "
                      "diagnostics) — running at default clock");
        return true;
    }

    if (otp_program_attempted_)
    {
        // Wrote earlier THIS boot and the post-reset verify still fails.
        // Do not write again — the NVS record below also blocks future boots.
        ESP_LOGE(TAG, "High-perf clock verify fails after this boot's OTP "
                      "write — NOT retrying (finite OTP)");
        return true;
    }

    // ── Once-EVER-per-module write guard ─────────────────────────────────
    // OTP capacity is a finite physical resource and u-blox does not
    // document whether repeated CFG-OTP writes consume additional space.
    // Policy: at most ONE programming attempt per physical module, tracked
    // in FC NVS against the receiver's unique chip ID (UBX-SEC-UNIQID).
    // No readable unique ID → no write, ever.
    const char *uniq = gnss.getUniqueChipIdStr(nullptr, 1100);
    if (uniq == nullptr || uniq[0] == '\0')
    {
        ESP_LOGE(TAG, "Cannot read module unique ID — refusing to write OTP "
                      "(no way to enforce the once-ever guard)");
        return true;
    }
    // Firmware-resident blocklist first: protects known failed/wedged modules
    // on ANY main board (a failed module reads BLANK, like factory-fresh).
    for (const char *blocked : kOtpNeverProgram)
    {
        if (strcmp(uniq, blocked) == 0)
        {
            ESP_LOGW(TAG, "Module %s is on the OTP never-program list — "
                          "running at default clock", uniq);
            return true;
        }
    }
    // NVS keys max 15 chars: "o" + up to 14 hex chars of the unique ID.
    char nvs_key[16] = {'o'};
    strncpy(nvs_key + 1, uniq, sizeof(nvs_key) - 2);
    nvs_key[sizeof(nvs_key) - 1] = '\0';

    Preferences prefs;
    if (!prefs.begin("gnssotp", false))
    {
        ESP_LOGE(TAG, "NVS unavailable — refusing to write OTP without the "
                      "once-ever guard");
        return true;
    }
    const uint8_t prior_attempts = prefs.getUChar(nvs_key, 0);
    if (prior_attempts != 0)
    {
        prefs.end();
        ESP_LOGE(TAG, "OTP write was ALREADY attempted on module %s (NVS "
                      "guard) but verify fails — NOT rewriting. Investigate "
                      "manually before clearing NVS key gnssotp/%s",
                 uniq, nvs_key);
        return true;
    }
    // Record the attempt BEFORE sending anything, so a crash/brownout
    // mid-write can never lead to a second automatic attempt.
    prefs.putUChar(nvs_key, (uint8_t)(prior_attempts + 1));
    prefs.end();
    otp_program_attempted_ = true;

    // §2.1.5 Table 3 configuration string as ONE CONTIGUOUS BURST, byte-exact
    // including sync chars and checksums (validated against the manual's own
    // published checksum bytes).  Bench 7/07 established that the two frames
    // sent as separate ACK-interleaved messages always fail: frame 1 ACKs but
    // a standalone frame 2 is NACKed at ANY spacing (15 ms and 300–900 ms
    // both tried) — CFG-OTP evidently treats the full string as one
    // transaction, matching the manual's flow ("send the configuration
    // string" ... "the device returns two UBX-ACK-ACK"): u-center transmits
    // the whole string first, then both ACKs come back.  The library can only
    // send one message per ACK round-trip, so this transaction bypasses it:
    // write the burst raw on the UART and scan the RX stream for the two
    // ACK-ACK sequences directly (NMEA chatter may interleave; the pattern
    // matcher tolerates it).
    ESP_LOGW(TAG, "Programming high-performance clock into OTP of module %s "
                  "(single burst attempt, one-time, permanent)", uniq);

    static const uint8_t kOtpBurst[] = {
        0xB5, 0x62, 0x06, 0x41, 0x10, 0x00,
        0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E, 0x79, 0xBF,
        0x28, 0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF,
        0x8F, 0x0D,
        0xB5, 0x62, 0x06, 0x41, 0x1C, 0x00,
        0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34, 0xF9, 0x12,
        0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40,
        0x00, 0xB0, 0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40,
        0x00, 0xD8, 0xB8, 0x05,
        0xDE, 0xAE,
    };
    // UBX-ACK-ACK / UBX-ACK-NAK for cls 0x06 id 0x41 (checksums recomputed
    // and cross-checked against the manual's published ACK sequence).
    static const uint8_t kAck[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
                                   0x06, 0x41, 0x4F, 0x78};
    static const uint8_t kNak[] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00,
                                   0x06, 0x41, 0x4E, 0x73};

    // Drain stale RX so the scan starts clean.
    {
        uint8_t tmp;
        while (uart_read_bytes(_uartPort, &tmp, 1, 0) > 0) {}
    }

    uart_write_bytes(_uartPort, kOtpBurst, sizeof(kOtpBurst));

    uint8_t acks = 0, naks = 0;
    size_t m_ack = 0, m_nak = 0;
    const uint32_t scan_start = millis();
    while ((millis() - scan_start) < 2500U && acks < 2)
    {
        uint8_t c;
        if (uart_read_bytes(_uartPort, &c, 1, pdMS_TO_TICKS(20)) != 1)
        {
            continue;
        }
        m_ack = (c == kAck[m_ack]) ? m_ack + 1 : ((c == kAck[0]) ? 1 : 0);
        if (m_ack == sizeof(kAck)) { acks++; m_ack = 0; }
        m_nak = (c == kNak[m_nak]) ? m_nak + 1 : ((c == kNak[0]) ? 1 : 0);
        if (m_nak == sizeof(kNak)) { naks++; m_nak = 0; }
    }

    if (acks == 2 && naks == 0)
    {
        ESP_LOGW(TAG, "OTP high-clock config written — both ACKs received");
        return false;  // caller resets the receiver and re-verifies once
    }
    ESP_LOGE(TAG, "OTP burst result: %u ACK, %u NAK (need 2 ACK / 0 NAK) — "
                  "locked out on this board by the NVS guard. ADD module %s "
                  "to kOtpNeverProgram so no other main board retries it.",
             acks, naks, uniq);
    return true;  // no reset loop; continue at current clock
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
    // #572: every getter passes maxWait=0 explicitly. The SparkFun default is
    // ~1.1 s: a getter whose cache bit is stale silently issues a BLOCKING
    // getPVT(1100) — safe on the pollNewPVT path only because getPVT(0)==true
    // just primed the whole cache, but a lethal footgun for any standalone
    // caller (up to ~1.1 s PER FIELD). With 0 the contract is explicit:
    // this function reads the already-parsed NAV-PVT and never touches the
    // serial link. Must be called after a successful getPVT (pollNewPVT does).
    gnss_data.time_us = micros();
    gnss_data.year = gnss.getYear(0);
    gnss_data.month = gnss.getMonth(0);
    gnss_data.day = gnss.getDay(0);
    gnss_data.hour = gnss.getHour(0);
    gnss_data.minute = gnss.getMinute(0);
    gnss_data.second = gnss.getSecond(0);
    gnss_data.milli_second = gnss.getMillisecond(0);
    // 0: No Fix, 1: Dead Reckoning, 2: 2D Fix, 3: 3D Fix,
    // 4:GNSS + Dead Reckoning, 5: Time Only
    gnss_data.fix_mode = gnss.getFixType(0);

    // #562: fixType alone is not enough to trust a fix. u-blox marks a fix
    // INVALID without dropping fixType below 3 — by clearing gnssFixOK (fix
    // outside the configured DOP/accuracy masks) or setting invalidLlh
    // (lat/lon/height not valid). gnssFixOK is ALSO how it signals a COCOM
    // violation (>515 m/s or >18 km) — exactly the transonic / high-altitude
    // regime the apogee GPS voter and guidance/landing-prediction run in. Zero
    // fix_mode so every downstream `fix_mode >= 3` consumer treats it as "no
    // fix". maxWait=0 keeps this non-blocking: both flags come from the
    // NAV-PVT that pollNewPVT already parsed this cycle. See open #491
    // (COCOM bench-test).
    if (!gnss.getGnssFixOk(0) || gnss.getInvalidLlh(0))
    {
        gnss_data.fix_mode = 0;
    }

    gnss_data.num_sats = gnss.getSIV(0);

    // SparkFun u-blox returns PDOP as scale 0.01. Convert to x10 for packed type.
    const uint16_t pdop_x100 = gnss.getPDOP(0);
    uint16_t pdop_x10_u16 = (uint16_t)((pdop_x100 + 5U) / 10U);
    if (pdop_x10_u16 > 255U) pdop_x10_u16 = 255U;
    gnss_data.pdop_x10 = (uint8_t)pdop_x10_u16;

    // Accuracy estimates are reported in mm. Convert to whole meters.
    const uint32_t h_acc_mm = gnss.getHorizontalAccEst(0);
    const uint32_t v_acc_mm = gnss.getVerticalAccEst(0);
    uint32_t h_acc_m_u32 = (h_acc_mm + 500U) / 1000U;
    uint32_t v_acc_m_u32 = (v_acc_mm + 500U) / 1000U;
    if (h_acc_m_u32 > 255U) h_acc_m_u32 = 255U;
    if (v_acc_m_u32 > 255U) v_acc_m_u32 = 255U;
    gnss_data.h_acc_m = (uint8_t)h_acc_m_u32;
    gnss_data.v_acc_m = (uint8_t)v_acc_m_u32;

    // Velocity (ENU, mm/s)
    gnss_data.vel_e_mmps = gnss.getNedEastVel(0);
    gnss_data.vel_n_mmps = gnss.getNedNorthVel(0);
    gnss_data.vel_u_mmps = -gnss.getNedDownVel(0);

    // Latitude and Longitude (deg*1e7)
    gnss_data.lat_e7 = gnss.getLatitude(0);
    gnss_data.lon_e7 = gnss.getLongitude(0);

    // Altitude relative to mean sea level (mm)
    gnss_data.alt_mm = gnss.getAltitudeMSL(0);

    // #572: the "staleness detection" that lived here was DEAD CODE — this
    // function is only reached from pollNewPVT() after getPVT(0) returned
    // true (a NEW NAV-PVT was fully parsed), so the second+millisecond
    // "unchanged across consecutive reads" condition it counted could never
    // occur on the live path; in the overflow/desync case it targeted,
    // getPVT(0) returns false and this function is never called. Genuine
    // GNSS silence is caught downstream by the collector/EKF time_us
    // freshness gates (and #557's absent-module handling). Removed rather
    // than relocated — a wall-clock silence detector here would duplicate
    // the downstream gates.
}

#if defined(TR_GNSS_COCOM_DIAG) && TR_GNSS_COCOM_DIAG

bool TR_GNSSReceiverUBloxSerial::enableSatDiag()
{
    // Automatic reports, so getNAVSAT(0) below never blocks the flight loop.
    const bool ok = gnss.setAutoNAVSAT(true);
    ESP_LOGI("GNSS", "[COCOM] NAV-SAT auto reports %s", ok ? "enabled" : "REFUSED");
    return ok;
}

void TR_GNSSReceiverUBloxSerial::logSatDiag()
{
    // Fix state first, straight from the library's NAV-PVT cache. Velocity is
    // logged as NED components rather than ground speed on purpose: COCOM acts
    // on 3-D speed, and a rocket's velocity is almost entirely vertical, so
    // ground speed reads near zero through exactly the part of a flight the
    // limit is about.
    ESP_LOGI("GNSS",
             "[COCOM] P tow=%lu fix=%u ok=%u nsv=%u lat=%ld lon=%ld alt=%ld "
             "vn=%ld ve=%ld vd=%ld",
             (unsigned long)gnss.getTimeOfWeek(0),
             (unsigned)gnss.getFixType(0),
             (unsigned)(gnss.getGnssFixOk(0) ? 1 : 0),
             (unsigned)gnss.getSIV(0),
             (long)gnss.getLatitude(0),
             (long)gnss.getLongitude(0),
             (long)gnss.getAltitude(0),
             (long)gnss.getNedNorthVel(0),
             (long)gnss.getNedEastVel(0),
             (long)gnss.getNedDownVel(0));

    if (!gnss.getNAVSAT(0) || gnss.packetUBXNAVSAT == nullptr)
        return;

    const uint8_t n = gnss.packetUBXNAVSAT->data.header.numSvs;
    // One line, space-separated gnss:sv:cno:used:elev. Kept on a single line so
    // a capture cannot interleave one epoch's satellites with another's under
    // the log mutex.
    //
    // Elevation is here because Doppler rate goes as a*sin(elevation) -- about
    // 693 Hz/s toward zenith against 60 Hz/s near the horizon on a 13.5 g
    // boost -- so which satellites a receiver drops under acceleration is only
    // answerable per-satellite with an elevation beside the C/N0. On the parts
    // that could be tapped directly, the high-elevation satellites are exactly
    // the ones lost through the burn; this receiver could not be checked at all
    // because the field was missing. Signed: NAV-SAT reports negative elevation
    // for satellites below the horizon.
    char line[640];
    int off = snprintf(line, sizeof(line), "[COCOM] S n=%u", (unsigned)n);
    for (uint8_t i = 0; i < n && off > 0 && off < (int)sizeof(line) - 20; i++)
    {
        const auto &b = gnss.packetUBXNAVSAT->data.blocks[i];
        off += snprintf(line + off, sizeof(line) - off, " %u:%u:%u:%u:%d",
                        (unsigned)b.gnssId, (unsigned)b.svId,
                        (unsigned)b.cno, (unsigned)(b.flags.bits.svUsed ? 1 : 0),
                        (int)b.elev);
    }
    ESP_LOGI("GNSS", "%s", line);
}

#endif  // TR_GNSS_COCOM_DIAG
