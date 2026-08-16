// TR_GNSSReceiverLC86_Serial.cpp — Quectel LC86G (LA) over ESP-IDF UART.
//
// Structure deliberately mirrors TR_GNSSReceiverUBlox_Serial.cpp (UART
// helpers, deadline-bounded begin, poll/get split) so fixes cross-apply;
// everything u-blox-protocol-specific (UBX handshakes, OTP clock programming,
// baud standardization to 460800, RX/TX orientation probe) is intentionally
// absent — see the port notes in scratchpad/report-lc86g-research.md.
//
// Protocol references in comments are to the Quectel LC26G/LC76G/LC86G GNSS
// Protocol Specification V1.4 [A] and the LC86G Hardware Design V1.1 [B].
#include "TR_GNSSReceiverLC86_Serial.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdio>
#include <cstring>

static const char* TAG = "GNSS";

static constexpr size_t GNSS_UART_RX_BUF = 4096;
static constexpr size_t GNSS_UART_TX_BUF = 256;

// LC86G power-on UART default is 115200 8N1 [A §2.4.69 recommends staying
// there; below 115200 "messages may be lost"]. Runtime policy is 115200-STAY:
// we never re-baud a healthy module — $PAIR864 needs a module reboot to fully
// apply [A §2.4.69], and 115200 carries our ~260 B/epoch at 10 Hz with >75%
// margin. This is the opposite of the u-blox driver's standardize-to-460800
// dance, on purpose.
static constexpr uint32_t kDefaultBaud = 115200U;

// Recovery probe order for a module someone reconfigured and $PAIR513-saved
// (config persists in module flash across our rail cycles; ours never saves).
static constexpr uint32_t kProbeBauds[] = {9600U, 460800U, 38400U, 230400U, 57600U};

// Bring-up deadline (#557 house rule): begin() must return false on a dead or
// deaf module rather than hang boot. The LC86G's time-to-first-NMEA after
// power-on is NOT documented ([B] fig. 10 leaves the UART "Invalid→Valid"
// transition unannotated), so the budget is generous: worst case is
// 3 s default listen + 5×1.2 s probe sweep + $PAIR864 apply + one more
// 3 s listen ≈ 12.5 s; 15 s covers one full recovery cycle. A healthy module
// exits the first listen in well under a second.
static constexpr uint32_t kBeginTimeoutMs  = 15000U;
static constexpr uint32_t kFirstSentenceMs = 3000U;
// A module at its 1 Hz factory fix rate bursts once per second — the probe
// window must exceed one full output period to catch a burst regardless of
// phase (same lesson as the u-blox driver's kOrientationProbeMs, bench
// 2026-05-29).
static constexpr uint32_t kProbeWindowMs = 1200U;

// $PAIR001 / $PQTMCFGMSGRATE ack policy: 500 ms wait, 2 retries (spec'd in
// the mini firmware plan; acks normally arrive within one fix interval).
static constexpr uint32_t kAckTimeoutMs = 500U;
static constexpr uint8_t  kAckRetries   = 2U;

// Post-config sanity window: PQTMPVT was ACKed, so at any legal fix interval
// (100–1000 ms) the first epoch must land well inside this.
static constexpr uint32_t kFirstPvtWaitMs = 1500U;

// Per-call RX drain cap for pollNewPVT(). The collector's poll task counts
// every call over 1 ms (pt_gnss_over_1ms) — the u-blox lesson was that
// draining a whole multi-constellation NMEA epoch in one call cost ~10 ms and
// starved IMU/baro polling. 512 B is ~2 epochs of our trimmed sentence set,
// so steady state never hits the cap; only a backlog (boot, task stall) does,
// and then line reassembly continues across calls.
static constexpr size_t kMaxBytesPerPoll = 512U;

static inline int64_t nowUs() { return esp_timer_get_time(); }

// Constructor
TR_GNSSReceiverLC86Serial::TR_GNSSReceiverLC86Serial(uart_port_t uart_port)
    : _uartPort(uart_port) {}

// ── UART helpers (ported verbatim from the u-blox driver) ───────────────

void TR_GNSSReceiverLC86Serial::uartBegin(uint32_t baud, uint8_t rx_pin, uint8_t tx_pin)
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

    // Bytes sampled at the old baud may leave a half-assembled line behind;
    // drop it so cross-baud garbage can't glue onto the next real sentence.
    parser_.resetLine();
}

void TR_GNSSReceiverLC86Serial::uartEnd()
{
    if (uart_is_driver_installed(_uartPort))
    {
        uart_driver_delete(_uartPort);
    }
}

// ── Command / ack plumbing (begin()-time only) ──────────────────────────

void TR_GNSSReceiverLC86Serial::sendSentence(const char* body)
{
    char frame[96];
    const size_t len = lc86::buildSentence(body, frame, sizeof(frame));
    if (len == 0)
    {
        ESP_LOGE(TAG, "Sentence too long to frame: %s", body);
        return;
    }
    uart_write_bytes(_uartPort, frame, len);
    // Commands are tiny (<40 B ≈ 3.5 ms at 115200, ~40 ms at 9600); wait for
    // the wire so a follow-on uartBegin() (baud recovery path) can never
    // truncate a command mid-frame when the driver is torn down.
    uart_wait_tx_done(_uartPort, pdMS_TO_TICKS(100));
}

bool TR_GNSSReceiverLC86Serial::awaitEvent(AwaitKind kind, uint32_t timeout_ms,
                                           uint16_t pair_id,
                                           uint8_t* pair_result,
                                           bool* qtm_ok)
{
    const int64_t deadline_us = nowUs() + (int64_t)timeout_ms * 1000;
    while (nowUs() < deadline_us)
    {
        uint8_t buf[64];
        const int n = uart_read_bytes(_uartPort, buf, sizeof(buf),
                                      pdMS_TO_TICKS(20));
        if (n <= 0) continue;
        for (int i = 0; i < n; i++)
        {
            const lc86::Event ev = parser_.feed(buf[i]);
            if (ev == lc86::Event::NONE || ev == lc86::Event::BAD) continue;

            // Epochs seen while waiting for something else still refresh the
            // cache (a module already configured by a previous begin() in
            // this power cycle streams PVT throughout the config phase).
            if (ev == lc86::Event::PVT) publishFromParser();

            switch (kind)
            {
                case AwaitKind::ANY_VALID:
                    return true;  // any checksummed sentence proves liveness

                case AwaitKind::PAIR_ACK:
                    if (ev == lc86::Event::PAIR_ACK &&
                        parser_.ackCommandId() == pair_id)
                    {
                        // Result 1 = "being processed" [A §2.4.1] — keep
                        // waiting for the final verdict.
                        if (parser_.ackResult() == 1) break;
                        if (pair_result) *pair_result = parser_.ackResult();
                        return true;
                    }
                    break;

                case AwaitKind::QTM_RESULT:
                    if (ev == lc86::Event::QTM_OK)
                    {
                        if (qtm_ok) *qtm_ok = true;
                        return true;
                    }
                    if (ev == lc86::Event::QTM_ERROR)
                    {
                        if (qtm_ok) *qtm_ok = false;
                        return true;
                    }
                    break;

                case AwaitKind::PVT:
                    if (ev == lc86::Event::PVT) return true;
                    break;
            }
        }
    }
    return false;
}

bool TR_GNSSReceiverLC86Serial::sendPairCommand(const char* body, uint16_t ack_id)
{
    // $PAIR001 acks carry only the command ID, so consecutive commands with
    // the same ID (the six $PAIR062 writes) could alias a late ack from the
    // previous send. Sends are strictly sequential with a full ack wait
    // between them, so the only alias window is a retry racing its own late
    // ack — accepted: every command here is idempotent and reapplied on every
    // begin() anyway.
    for (uint8_t attempt = 0; attempt <= kAckRetries; attempt++)
    {
        sendSentence(body);
        uint8_t result = 0xFF;
        if (awaitEvent(AwaitKind::PAIR_ACK, kAckTimeoutMs, ack_id, &result))
        {
            if (result == 0) return true;
            // Result 4 (parameter error) won't improve with retries; 5 (MNL
            // busy) will. The uniform retry keeps this simple — 2 extra
            // idempotent sends at worst.
            ESP_LOGW(TAG, "$%s: PAIR001 result %u", body, result);
        }
        else
        {
            ESP_LOGW(TAG, "$%s: no PAIR001 ack in %lu ms", body,
                     (unsigned long)kAckTimeoutMs);
        }
    }
    return false;
}

bool TR_GNSSReceiverLC86Serial::sendQtmMsgRate(const char* body)
{
    // $PQTMCFGMSGRATE acks are NOT PAIR001 — they are $PQTMCFGMSGRATE,OK or
    // $PQTMCFGMSGRATE,ERROR,<code> [A §2.3.1].
    for (uint8_t attempt = 0; attempt <= kAckRetries; attempt++)
    {
        sendSentence(body);
        bool ok = false;
        if (awaitEvent(AwaitKind::QTM_RESULT, kAckTimeoutMs, 0, nullptr, &ok))
        {
            if (ok) return true;
            ESP_LOGW(TAG, "$%s: PQTMCFGMSGRATE ERROR %d", body,
                     parser_.qtmErrorCode());
        }
        else
        {
            ESP_LOGW(TAG, "$%s: no PQTMCFGMSGRATE response in %lu ms", body,
                     (unsigned long)kAckTimeoutMs);
        }
    }
    return false;
}

// ── begin() ─────────────────────────────────────────────────────────────

bool TR_GNSSReceiverLC86Serial::begin(uint8_t update_rate_hz_in,
                                      uint8_t GNSS_RX,
                                      uint8_t GNSS_TX,
                                      int8_t reset_n_pin,
                                      int8_t safeboot_n_pin)
{
    // Signature compatibility only (the collector passes both pins
    // unconditionally): the LC86G has no SAFEBOOT concept, and the mini does
    // not wire RESET_N — the module self-starts when its rail rises
    // ([B §4.2.1]: RESET_N is only needed to leave Backup mode, which we
    // never enter; the rail cycle IS our reset).
    (void)reset_n_pin;
    (void)safeboot_n_pin;

    // LC86G (LA) fix-rate ceiling is 10 Hz (datasheet: "1 Hz default,
    // max. 10 Hz"); $PAIR050 takes an interval of 100–1000 ms [A §2.4.10].
    update_rate_hz = update_rate_hz_in;
    if (update_rate_hz < 1) update_rate_hz = 1;
    if (update_rate_hz > 10)
    {
        ESP_LOGW(TAG, "Requested %u Hz exceeds LC86G max; clamping to 10 Hz",
                 update_rate_hz_in);
        update_rate_hz = 10;
    }

    pvt_stream_ok_ = false;

    ESP_LOGI(TAG, "Starting LC86G configuration...");

    // ── Liveness (#557 analogue) ────────────────────────────────────────
    // Wait for the first CHECKSUM-VALID sentence — a bare bytes-available
    // check is what burned the u-blox bring-up (noise/wrong-baud bytes trip
    // it); a verified checksum can only come from a live module at the right
    // baud. Deadline-bounded so a dead/deaf module degrades instead of
    // hanging boot.
    const int64_t begin_deadline_us = nowUs() + (int64_t)kBeginTimeoutMs * 1000;

    uartBegin(kDefaultBaud, GNSS_RX, GNSS_TX);

    bool alive = false;
    while (!alive)
    {
        if (awaitEvent(AwaitKind::ANY_VALID, kFirstSentenceMs))
        {
            alive = true;
            break;
        }
        if (nowUs() >= begin_deadline_us) break;

        // Nothing valid at the power-on default. Either the module is still
        // booting (start-talking time is undocumented — the outer loop keeps
        // re-listening) or someone reconfigured+saved a different baud: probe
        // the plausible set and command such a module back to 115200.
        const size_t n_bauds = sizeof(kProbeBauds) / sizeof(kProbeBauds[0]);
        for (size_t i = 0; i < n_bauds; i++)
        {
            if (nowUs() >= begin_deadline_us) break;
            ESP_LOGW(TAG, "No NMEA at %lu baud; probing %lu",
                     (unsigned long)kDefaultBaud, (unsigned long)kProbeBauds[i]);
            uartBegin(kProbeBauds[i], GNSS_RX, GNSS_TX);
            if (!awaitEvent(AwaitKind::ANY_VALID, kProbeWindowMs)) continue;

            ESP_LOGW(TAG, "Module talking at %lu baud — commanding back to "
                          "115200 ($PAIR864)", (unsigned long)kProbeBauds[i]);
            sendSentence("PAIR864,0,0,115200");
            // [A §2.4.69] says a reboot is needed for the new baud to take
            // effect; in practice the port re-rates immediately. Cover both:
            // reopen at 115200 and let the outer loop re-listen — if the
            // module ignored the change, the next probe sweep finds it again
            // (all bounded by the begin deadline).
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
        // Whatever the sweep found (or didn't), the next outer-loop listen
        // must run at the power-on default — a slow-to-start healthy module
        // would otherwise be listened for at the LAST probe baud for the
        // rest of the deadline and never heard (kProbeBauds deliberately
        // excludes 115200).
        uartBegin(kDefaultBaud, GNSS_RX, GNSS_TX);
        if (nowUs() >= begin_deadline_us) break;
    }

    if (!alive)
    {
        ESP_LOGE(TAG, "GNSS bring-up: no valid NMEA after %lu ms; "
                      "continuing without GNSS", (unsigned long)kBeginTimeoutMs);
        return false;
    }

    ESP_LOGI(TAG, "LC86G alive at %lu baud", (unsigned long)kDefaultBaud);

    // ── Sentence selection ($PAIR062 [A §2.4.14]) ───────────────────────
    // Strip the default NMEA chatter (GGA/GLL/GSA/GSV/RMC/VTG all default
    // on) down to GGA-only: the poll task's <1 ms latency budget is why —
    // see kMaxBytesPerPoll. GGA is kept at the fix rate as a cheap liveness/
    // debug channel (sat count before the first PVT). None of these are
    // load-bearing, so failures WARN and continue.
    static const struct { const char* body; const char* what; } kSentenceSel[] = {
        {"PAIR062,1,0", "GLL off"},
        {"PAIR062,3,0", "GSV off"},
        {"PAIR062,2,0", "GSA off"},
        {"PAIR062,5,0", "VTG off"},
        {"PAIR062,0,1", "GGA at fix rate"},
        {"PAIR062,4,0", "RMC off"},
    };
    for (const auto& cmd : kSentenceSel)
    {
        if (!sendPairCommand(cmd.body, 62))
        {
            ESP_LOGW(TAG, "Sentence config failed: %s (continuing)", cmd.what);
        }
    }

    // ── $PQTMPVT enable — THE load-bearing config ───────────────────────
    // PQTMPVT is the only sentence this module emits with vertical velocity
    // (VelD) [A §2.3.8]. MsgVer must be given explicitly or the module
    // returns an error [A §2.3.1 note]; PVT's MsgVer is 1.
    if (!sendQtmMsgRate("PQTMCFGMSGRATE,W,PQTMPVT,1,1"))
    {
        // NEVER fall back to GGA/RMC and synthesize vel_d: the EKF hard-fuses
        // velD at sigma 0.3 m/s, so a synthesized 0 would be TRUSTED and
        // poison the altitude channel through the whole flight. Treat the
        // module as absent instead — the insurance latch below keeps
        // fix_mode=0 on anything that polls regardless, and returning false
        // puts the collector in its proven GNSS-absent degraded mode (#557):
        // baro-backstop apogee, baro deploy, EKF from baro+IMU.
        pvt_stream_ok_ = false;
        ESP_LOGE(TAG, "PQTMPVT enable FAILED — no vertical-velocity source; "
                      "treating GNSS as absent");
        return false;
    }
    pvt_stream_ok_ = true;

    // ── $PQTMEPE enable — accuracy estimates (nice-to-have) ─────────────
    // Supplies h_acc/v_acc (EPE_2D/EPE_Down); PQTMPVT carries no accuracy
    // fields. The doc's $PQTMEPE message version is 2; some firmware may
    // want 1, so fall back once. Tolerable failure: accuracies then read
    // 255 m (= unknown/worst) — flag for the coordinator's EKF gate review.
    if (!sendQtmMsgRate("PQTMCFGMSGRATE,W,PQTMEPE,1,2") &&
        !sendQtmMsgRate("PQTMCFGMSGRATE,W,PQTMEPE,1,1"))
    {
        ESP_LOGW(TAG, "PQTMEPE enable failed — h/v accuracy will read 255 m");
    }

    // ── Fix rate ($PAIR050 [A §2.4.10]) ─────────────────────────────────
    // Interval in ms, legal range 100–1000 (10 Hz → 100). At >1 Hz the
    // module keeps only RMC/GGA/GNS at rate and pins GSA/GSV to 1 Hz — we
    // already turned those off. LC86G (LA) supports this command (only the
    // -T/PA variants don't).
    {
        char rate_body[24];
        const unsigned interval_ms = 1000U / update_rate_hz;
        snprintf(rate_body, sizeof(rate_body), "PAIR050,%u", interval_ms);
        if (!sendPairCommand(rate_body, 50))
        {
            ESP_LOGW(TAG, "Fix-rate set failed — module stays at its prior "
                          "rate (data still flows, just slower)");
        }
    }

    // Deliberately NO $PAIR513 / $PQTMSAVEPAR: config is reapplied on every
    // begin() — a rail cycle wipes the RTC-RAM config anyway (V_BCKP rides
    // the same switched rail), and a flash save at >1 Hz requires the
    // documented $PAIR382/$PAIR003 GNSS power-down dance [A §2.4.49 note 2]
    // we do not want anywhere near a pad power-on.

    // ── Post-config sanity ──────────────────────────────────────────────
    // PQTMPVT was ACKed and emits at the fix interval regardless of fix
    // state, so silence here means the ack lied or PQTM output is throttled
    // at >1 Hz fix rates (undocumented either way — research-report caveat).
    // WARN for the bench; the ack is authoritative per the plan, and the
    // collector/EKF freshness gates handle a silent stream downstream.
    if (awaitEvent(AwaitKind::PVT, kFirstPvtWaitMs))
    {
        ESP_LOGI(TAG, "First $PQTMPVT epoch observed");
    }
    else
    {
        ESP_LOGW(TAG, "No $PQTMPVT within %lu ms of enable — bench-verify "
                      "PQTM output at %u Hz",
                 (unsigned long)kFirstPvtWaitMs, update_rate_hz);
    }

    ESP_LOGI(TAG, "Configuration complete.");
    return true;
}

// ── Poll path ───────────────────────────────────────────────────────────

void TR_GNSSReceiverLC86Serial::publishFromParser()
{
    data_ = parser_.pvt();
    // MCU sample time, not GNSS time — identical semantics to the u-blox
    // driver; downstream freshness gates key off this.
    data_.time_us = (uint32_t)esp_timer_get_time();
    if (!pvt_stream_ok_)
    {
        // Insurance latch for the vel_d rule: if begin() never got PQTMPVT
        // accepted it returned false and nothing should be polling — but if
        // anything does, no epoch may masquerade as a usable fix.
        data_.fix_mode = 0;
    }
    pvt_epochs_++;
}

bool TR_GNSSReceiverLC86Serial::pollNewPVT(GNSSData &gnss_data)
{
    // Incremental, latency-bounded drain: at most kMaxBytesPerPoll bytes per
    // call, line reassembly continues across calls, every line checksum-
    // verified by the parser. Non-blocking (read timeout 0).
    uint8_t buf[64];
    size_t drained = 0;
    bool new_pvt = false;

    while (drained < kMaxBytesPerPoll)
    {
        size_t want = kMaxBytesPerPoll - drained;
        if (want > sizeof(buf)) want = sizeof(buf);
        const int n = uart_read_bytes(_uartPort, buf, want, 0);
        if (n <= 0) break;
        drained += (size_t)n;

        for (int i = 0; i < n; i++)
        {
            const lc86::Event ev = parser_.feed(buf[i]);
            if (ev == lc86::Event::PVT)
            {
                // Two epochs in one call (backlog drain) publish the newest
                // and still return true once — downstream wants the latest
                // epoch, not a replay.
                publishFromParser();
                new_pvt = true;
            }
            else if (ev == lc86::Event::GGA && pvt_epochs_ == 0)
            {
                // Pre-first-PVT debug visibility only: sat count while
                // acquiring. Never a fix source (cache fix_mode stays 0).
                data_.num_sats = parser_.ggaNumSats();
            }
        }
    }

    if (!new_pvt) return false;  // `gnss_data` untouched (u-blox contract)
    getGNSSData(gnss_data);
    return true;
}

void TR_GNSSReceiverLC86Serial::getGNSSData(GNSSData &gnss_data)
{
    // Reads the already-parsed cache and NEVER touches the serial link (#572
    // contract carried over from the u-blox driver, where a stale cache bit
    // could silently issue a ~1.1 s blocking read per field). Only meaningful
    // immediately after a successful pollNewPVT().
    gnss_data = data_;
}
