#include "modem_config_ack.h"
#include <math.h>
#include "UartModemBackend.h"

#include <cstring>

#include <compat.h>
#include <driver/gpio.h>
#include <driver/uart.h>  // uart_flush_input on the re-attach probe
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "RADIO_MODEM";

using namespace radio_modem;

// ---------------------------------------------------------------------------
// begin / config push
// ---------------------------------------------------------------------------

bool UartModemBackend::begin(const Config& cfg, float freq_mhz, uint8_t sf,
                             float bw_khz, uint8_t cr, int8_t tx_power,
                             bool debug)
{
    cfg_ = cfg;
    debug_ = debug;
    // Seed the cached radio params with the host's DESIRED config before we
    // know whether a modem is attached. If begin() times out (daughterboard
    // absent/unpowered at boot) but the modem shows up later, its BOOT frame
    // hot-joins: the handler below re-pushes these values instead of zeros.
    cfg_freq_mhz_ = freq_mhz;
    cfg_sf_ = sf;
    cfg_bw_khz_ = bw_khz;
    cfg_cr_ = cr;
    cfg_tx_power_ = tx_power;

    if (cfg.act_pin >= 0)
    {
        gpio_set_direction((gpio_num_t)cfg.act_pin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)cfg.act_pin, 1);  // power the daughterboard
    }

    // Every failure path below leaves the daughterboard UNPOWERED and the
    // backend not-began. Previously they all returned false with act_pin still
    // high and began_ already true, which cost two things: an unused 22 dBm
    // radio module sat powered for the rest of the flight, and service() kept
    // polling a link that begin() had just declared dead. LORA_ACT is also the
    // recovery hammer for a wedged modem (#412) — dropping it means a later
    // retry gets a genuine power cycle instead of re-poking a stuck part.
    //
    // #823: that "later retry" is now real.  It never was — begin() is called
    // exactly once per boot on both hosts, so dropping act_pin simply ended the
    // radio for the session, and the hot-join this function's own header
    // comment advertises could not fire either (unpowered module, and
    // service() returned on !began_ before polling).  armReattach() below
    // schedules the non-blocking re-attach in service(); see serviceReattach().
    //
    // `permanent` = the modem ANSWERED and is definitively incompatible.  Only
    // a non-responsive modem is worth re-attaching to: a protocol-version
    // mismatch is a mismatched daughterboard pair, and power-cycling it on a
    // timer forever cannot make it compatible — it would just keep dropping a
    // 22 dBm module in and out for the rest of the flight.
    auto failClosed = [&](bool permanent) -> bool
    {
        modem_alive_   = false;
        began_         = false;
        stats_.enabled = false;
        if (cfg.act_pin >= 0)
        {
            gpio_set_level((gpio_num_t)cfg.act_pin, 0);
        }
        if (permanent)
        {
            reattach_state_ = Reattach::Off;
            ESP_LOGE(TAG, "radio permanently disabled — not re-attaching");
        }
        else
        {
            armReattach(millis());
        }
        return false;
    };

    if (link_.begin(cfg.uart) != ESP_OK)
    {
        ESP_LOGE(TAG, "UART link init failed");
        return failClosed(false);
    }
    link_open_ = true;
    began_ = true;

    // The modem sends BOOT (identity payload) once its firmware is up —
    // S3 boot is ~1 s from the ACT edge; wait generously. If we attached to
    // an already-running modem (warm OC reboot), there is no BOOT coming, so
    // also poke GET_IDENTITY and accept either reply.
    // GET_IDENTITY is re-sent through the wait, not fired once: on hosts
    // without an ACT pin (the BS — J6 pin 1 is hard ground) an
    // already-running modem sends no BOOT, so a single lost/garbled request
    // frame would have been the difference between a radio and a radio-less
    // session until someone power-cycled the daughterboard.
    // 4 s is sized for the ACT-less host (the BS): there we are attaching to a
    // modem that may already have been running for hours, so the only way in is
    // to keep asking and hope one GET_IDENTITY lands. A host WITH an ACT pin
    // just powered the module itself and knows the S3 boots in ~1 s from that
    // edge, so 2.5 s is already 2.5x margin — and every extra second here is
    // paid inside an initPeripherals() that can already block 30-90 s, on the
    // exact path a user hits when the daughterboard is simply not plugged in.
    const uint32_t deadline = millis() + (cfg.act_pin >= 0 ? 2500 : 4000);
    uint32_t next_identity_ms = 0;
    while (millis() < deadline && !modem_alive_)
    {
        if ((int32_t)(millis() - next_identity_ms) >= 0)
        {
            (void)link_.sendFrame(MSG_GET_IDENTITY, nullptr, 0);
            next_identity_ms = millis() + 300;
        }
        link_.poll(&UartModemBackend::onFrameTrampoline, this);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!modem_alive_)
    {
        ESP_LOGE(TAG, "no modem answer on the UART link — radio disabled");
        return failClosed(false);
    }
    if (identity_.protocol_version != PROTOCOL_VERSION)
    {
        // Loud and fatal-for-the-radio: a mismatched matched-pair must not
        // limp along mis-parsing frames.
        ESP_LOGE(TAG, "modem protocol v%u != host v%u — radio disabled "
                      "(mismatched daughterboard pair?)",
                 identity_.protocol_version, PROTOCOL_VERSION);
        return failClosed(true);
    }
    ESP_LOGI(TAG, "modem up: chip=%u fw=%.32s max_tx=%ddBm band=%.0f-%.0fMHz",
             identity_.chip, identity_.fw_version,
             (int)identity_.max_tx_power_dbm,
             (double)identity_.freq_min_mhz, (double)identity_.freq_max_mhz);

    // A BOOT received during the wait loop above queues a deferred re-push;
    // moot now — begin() pushes the same config itself right here.
    config_repush_pending_ = false;
    if (!pushConfig(freq_mhz, sf, bw_khz, cr, tx_power, /*start_rx=*/true,
                    /*ack_timeout_ms=*/500))
    {
        // #569: covers both "no STATUS ack" and "ack with radio_enabled=0"
        // (RF dead on the daughterboard — pushConfig logs the specific cause).
        ESP_LOGE(TAG, "initial SET_CONFIG rejected — radio disabled");
        return failClosed(false);
    }

    stats_.enabled = true;
    ESP_LOGI(TAG, "radio configured via modem: %.1f MHz SF%u BW%.0f CR%u %ddBm",
             (double)freq_mhz, sf, (double)bw_khz, cr, (int)tx_power);
    return true;
}

bool UartModemBackend::pushConfig(float freq_mhz, uint8_t sf, float bw_khz,
                                  uint8_t cr, int8_t tx_power, bool start_rx,
                                  uint32_t ack_timeout_ms)
{
    RadioConfigData d = {};
    d.freq_mhz = freq_mhz;
    d.bandwidth_khz = bw_khz;
    d.spreading_factor = sf;
    d.coding_rate = cr;
    // Clamp host-side too (modem clamps as well — belt & braces for a
    // higher-power pair on one end only).
    d.tx_power_dbm = (identity_.max_tx_power_dbm != 0 &&
                      tx_power > identity_.max_tx_power_dbm)
                         ? identity_.max_tx_power_dbm
                         : tx_power;
    d.preamble_len = cfg_.preamble_len;
    d.flags = (cfg_.crc_on ? CFG_FLAG_CRC_ON : 0) |
              (cfg_.rx_boosted_gain ? CFG_FLAG_RX_BOOSTED_GAIN : 0) |
              (cfg_.syncword_private ? CFG_FLAG_SYNCWORD_PRIVATE : 0);
    d.start_rx = start_rx ? 1 : 0;

    const uint32_t status_before = status_rx_count_;
    if (!link_.sendFrame(MSG_SET_CONFIG, reinterpret_cast<uint8_t*>(&d),
                         sizeof(d)))
    {
        return false;
    }

    // The modem acks every SET_CONFIG with a STATUS frame.
    const uint32_t deadline = millis() + ack_timeout_ms;
    while (millis() < deadline)
    {
        link_.poll(&UartModemBackend::onFrameTrampoline, this);
        if (status_rx_count_ != status_before)
        {
            // #569: the STATUS ack proves the MODEM is alive — not the radio.
            // The modem echoes STATUS even when its LLCC68 begin() failed
            // (radio_enabled=0). Accepting that as success made begin()
            // report a fully working radio while every send() was silently
            // rejected modem-side (handleTxFrame: !radio_up → ok=0) — the OC
            // transmitted telemetry into a void, discoverable only by
            // watching tx_fail climb. Reject the config so begin() degrades
            // to radio-less, same as a failed TR_LoRa_Comms::begin(), and
            // the RF-dead state is loud and distinct from modem-absent.
            if (last_status_.radio_enabled == 0)
            {
                ESP_LOGE(TAG, "modem alive but RADIO DEAD (STATUS ack has "
                              "radio_enabled=0 — LLCC68 init failed on the "
                              "daughterboard) — rejecting config");
                return false;
            }
            // #835 item 7: radio_enabled only proves the radio is ALIVE.
            // A failed reconfigure() rolls TR_LoRa_Comms back to the previous
            // modulation and leaves it up, so the old code cached — and the
            // OC wrote to NVS — a modulation that was never on the air.  Next
            // boot then begin()s the illegal pair and disables the radio.
            // The rule itself lives in modem_config_ack.h so it can be unit
            // tested without a modem; see test_modem_config_ack.cpp.
            if (!modem_config_ack::accepted(
                    modem_config_ack::Ack{last_status_.config_ok,
                                          last_status_.current_freq_mhz,
                                          last_status_.current_sf},
                    modem_config_ack::Want{freq_mhz, sf}))
            {
                ESP_LOGE(TAG, "config NOT applied: asked %.3f MHz SF%u, modem "
                              "reports %.3f MHz SF%u (config_ok=%u) — radio is "
                              "on its previous modulation; not caching",
                         (double)freq_mhz, (unsigned)sf,
                         (double)last_status_.current_freq_mhz,
                         (unsigned)last_status_.current_sf,
                         (unsigned)last_status_.config_ok);
                return false;
            }
            cfg_freq_mhz_ = freq_mhz;
            cfg_sf_ = sf;
            cfg_bw_khz_ = bw_khz;
            cfg_cr_ = cr;
            cfg_tx_power_ = d.tx_power_dbm;
            // Any successful push satisfies a pending post-BOOT re-push —
            // e.g. a host-driven reconfigure() racing the BOOT handler.
            config_repush_pending_ = false;
            return true;
        }
        vTaskDelay(1);
    }
    return false;
}

// ---------------------------------------------------------------------------
// Frame dispatch
// ---------------------------------------------------------------------------

void UartModemBackend::onFrameTrampoline(void* ctx, uint8_t type,
                                         const uint8_t* payload, size_t len)
{
    static_cast<UartModemBackend*>(ctx)->onFrame(type, payload, len);
}

void UartModemBackend::onFrame(uint8_t type, const uint8_t* payload, size_t len)
{
    switch (type)
    {
        case MSG_RX_FRAME:
        {
            if (len <= sizeof(RxFrameHeader))
            {
                break;
            }
            RxFrameHeader hdr;
            memcpy(&hdr, payload, sizeof(hdr));
            const uint8_t* air = payload + sizeof(hdr);
            const size_t air_len = len - sizeof(hdr);
            if (air_len > MAX_AIR_FRAME)
            {
                break;
            }
            if (rxq_used_ >= RX_QUEUE_LEN)
            {
                // Oldest-drop: the OC polls every loop tick, so a full queue
                // means the loop stalled — keep the newest frames.
                rxq_head_ = (rxq_head_ + 1) % RX_QUEUE_LEN;
                rxq_used_--;
                stats_.rx_len_drop++;  // repurposed: queue-overflow drops
            }
            RxEntry& e = rx_queue_[(rxq_head_ + rxq_used_) % RX_QUEUE_LEN];
            e.len = static_cast<uint8_t>(air_len);
            e.rssi = hdr.rssi_dbm;
            e.snr = hdr.snr_db;
            memcpy(e.data, air, air_len);
            rxq_used_++;
            stats_.rx_count++;
            break;
        }

        case MSG_TX_RESULT:
        {
            if (len != sizeof(TxResultData))
            {
                break;
            }
            TxResultData r;
            memcpy(&r, payload, sizeof(r));
            if (tx_in_flight_ && r.seq == tx_seq_)
            {
                tx_in_flight_ = false;
                stats_.transmitting = false;
                if (r.ok)
                {
                    stats_.tx_ok++;
                }
                else
                {
                    stats_.tx_fail++;
                }
            }
            break;
        }

        case MSG_SCAN_RESULT:
        {
            if (len < sizeof(ScanResultHeader))
            {
                break;
            }
            ScanResultHeader hdr;
            memcpy(&hdr, payload, sizeof(hdr));
            size_t n = hdr.count;
            if (n > TR_LoRa_Comms::SCAN_MAX_SAMPLES)
            {
                n = TR_LoRa_Comms::SCAN_MAX_SAMPLES;
            }
            if (n > len - sizeof(hdr))
            {
                n = len - sizeof(hdr);
            }
            const uint8_t* rssi = payload + sizeof(hdr);
            for (size_t i = 0; i < n; i++)
            {
                scan_samples_[i].freq_mhz =
                    hdr.start_mhz + (float)i * hdr.step_khz / 1000.0f;
                scan_samples_[i].rssi_dbm = (int8_t)rssi[i];
            }
            scan_start_mhz_ = hdr.start_mhz;
            scan_step_khz_ = hdr.step_khz;
            scan_count_ = n;
            scan_active_ = false;
            scan_done_ = true;
            break;
        }

        case MSG_STATUS:
        {
            if (len != sizeof(ModemStatusData))
            {
                break;
            }
            memcpy(&last_status_, payload, sizeof(last_status_));
            status_rx_count_++;
            // Adopt link-health counters the host can't see locally
            stats_.rx_crc_fail = last_status_.rx_crc_fail;
            stats_.tx_watchdog_fires = last_status_.tx_watchdog_fires;
            break;
        }

        case MSG_IDENTITY:
        case MSG_BOOT:
        {
            if (len != sizeof(ModemIdentityData))
            {
                break;
            }
            memcpy(&identity_, payload, sizeof(identity_));
            // Version-check EVERY identity, not just the one begin() sees:
            // a BOOT can arrive because the daughterboard was reflashed or
            // physically swapped under a running host, and re-arming
            // modem_alive_ without the check would reintroduce the exact
            // mis-parsing begin() refuses (#569 class).
            if (identity_.protocol_version != PROTOCOL_VERSION)
            {
                ESP_LOGE(TAG, "modem protocol v%u != host v%u — radio stays "
                              "disabled (mismatched daughterboard pair?)",
                         identity_.protocol_version, PROTOCOL_VERSION);
                modem_alive_ = false;
                // The modem ANSWERED, so it is not "non-responsive" — the one
                // thing the re-attach is for.  Latch it so serviceReattach()
                // can tell an incompatible pair apart from silence: without
                // this, a mismatch found during a probe looks identical to no
                // reply and the module gets power-cycled every 60 s for the
                // rest of the flight.  begin() catches the mismatch it sees
                // directly (failClosed(true)); this covers every other path.
                modem_incompatible_ = true;
                break;
            }
            const bool was_alive = modem_alive_;
            modem_alive_ = true;
            if (type == MSG_IDENTITY) identity_reply_seen_ = true;
            if (type == MSG_BOOT)
            {
                // Daughterboard (re)booted underneath us: either it rebooted
                // (brownout / watchdog) or it hot-joined after a begin()
                // timeout. Its queue is empty and its radio is on boot
                // defaults — clear the in-flight slot, fail any active scan
                // (its SCAN_RESULT is never coming), and re-push our config.
                // The push is DEFERRED to service(): pushConfig() polls the
                // link, and calling it from inside this frame handler would
                // re-enter link_.poll() mid-chunk.
                ESP_LOGW(TAG, "modem %s — re-pushing radio config",
                         was_alive ? "REBOOTED" : "hot-joined");
                if (tx_in_flight_)
                {
                    tx_in_flight_ = false;
                    stats_.tx_fail++;
                }
                if (scan_active_)
                {
                    scan_active_ = false;
                    scan_done_ = true;
                    scan_count_ = 0;
                }
                config_repush_pending_ = true;
            }
            break;
        }

        default:
            if (debug_)
            {
                ESP_LOGW(TAG, "unknown modem msg 0x%02X (%u B)", type,
                         (unsigned)len);
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// IRadioLink surface
// ---------------------------------------------------------------------------

void UartModemBackend::armReattach(uint32_t now_ms)
{
    if (reattach_state_ == Reattach::Off && reattach_attempts_ == 0)
    {
        reattach_backoff_ms_ = REATTACH_FIRST_MS;
    }
    else
    {
        reattach_backoff_ms_ = (reattach_backoff_ms_ == 0)
                                   ? REATTACH_FIRST_MS
                                   : reattach_backoff_ms_ * 2;
        if (reattach_backoff_ms_ > REATTACH_MAX_MS) reattach_backoff_ms_ = REATTACH_MAX_MS;
    }
    reattach_at_ms_ = now_ms + reattach_backoff_ms_;
    reattach_state_ = Reattach::Backoff;
    ESP_LOGW(TAG, "modem re-attach armed — retry in %lu ms (attempt %u)",
             (unsigned long)reattach_backoff_ms_, (unsigned)(reattach_attempts_ + 1));
}

// Non-blocking re-attach.  Deliberately does NOT call begin(): that blocks
// 2.5-4 s in its identity wait, which is fine once during boot and unacceptable
// on a flying rocket's main loop.  The same sequence is spread across service()
// calls instead — power up, open the link if needed, poke GET_IDENTITY on a
// timer, and let the existing onFrame() handler set modem_alive_ off either a
// BOOT or an IDENTITY reply.
//
// Retrying in flight is safe precisely BECAUSE the radio is already dead: the
// module is unpowered and began_ is false, so there is no working link to
// disturb and nothing is transmitting.  A modem that is merely wedged rather
// than absent gets a genuine power cycle out of it, which is the #412 hammer.
void UartModemBackend::serviceReattach(uint32_t now_ms)
{
    switch (reattach_state_)
    {
        case Reattach::Off:
            return;

        case Reattach::Backoff:
        {
            if ((int32_t)(now_ms - reattach_at_ms_) < 0) return;
            reattach_attempts_++;
            if (cfg_.act_pin >= 0)
            {
                gpio_set_direction((gpio_num_t)cfg_.act_pin, GPIO_MODE_OUTPUT);
                gpio_set_level((gpio_num_t)cfg_.act_pin, 1);
            }
            // TR_UART_Link has no end(); re-installing the driver on an already
            // open port would fail, so only open it the first time.
            if (!link_open_)
            {
                if (link_.begin(cfg_.uart) != ESP_OK)
                {
                    ESP_LOGW(TAG, "re-attach: UART link init failed");
                    if (cfg_.act_pin >= 0) gpio_set_level((gpio_num_t)cfg_.act_pin, 0);
                    armReattach(now_ms);
                    return;
                }
                link_open_ = true;
            }
            modem_alive_          = false;
            modem_incompatible_   = false;
            identity_reply_seen_  = false;
            // Discard anything buffered from BEFORE this power cycle.  Without
            // it a stale IDENTITY still sitting in the driver's RX FIFO from
            // the previous attempt would satisfy the probe immediately, and we
            // would declare the module re-attached before it had even booted.
            uart_flush_input(cfg_.uart.port);
            // Same windows begin() uses, and for the same reason: a host with an
            // ACT pin just powered the module and knows the S3 boots in ~1 s,
            // while an ACT-less host (the BS) may be attaching to a module that
            // has been running for hours and can only keep asking.
            reattach_deadline_ms_    = now_ms + (cfg_.act_pin >= 0 ? 2500 : 4000);
            reattach_identity_at_ms_ = now_ms;
            reattach_state_          = Reattach::Probing;
            return;
        }

        case Reattach::Probing:
        {
            if ((int32_t)(now_ms - reattach_identity_at_ms_) >= 0)
            {
                (void)link_.sendFrame(MSG_GET_IDENTITY, nullptr, 0);
                reattach_identity_at_ms_ = now_ms + 300;
            }
            link_.poll(&UartModemBackend::onFrameTrampoline, this);

            // Require an IDENTITY *reply*, not merely modem_alive_.  An
            // unsolicited BOOT sets modem_alive_ too, but it only proves the
            // modem can transmit — if our TX direction is dead, SET_CONFIG will
            // never land, and declaring re-attach here would set began_ and so
            // stop serviceReattach() from ever power-cycling again, stranding a
            // half-attached link with no recovery.  A reply to our own
            // GET_IDENTITY proves both directions, and costs at most one 300 ms
            // poke to obtain.
            if (modem_alive_ && identity_reply_seen_)
            {
                // onFrame() already rejected a protocol mismatch by clearing
                // modem_alive_, so reaching here means a compatible modem
                // answered.  Do NOT push config from here — hand off to the
                // durable config_repush_pending_ path in service() below, which
                // already retries on a timer and already covers the RF-dead
                // modem that acks with radio_enabled=0.
                began_                 = true;
                stats_.enabled         = true;
                config_repush_pending_ = true;
                repush_retry_at_ms_    = now_ms;
                reattach_state_        = Reattach::Off;
                // Reset the backoff so a LATER arm starts at REATTACH_FIRST_MS
                // again rather than inheriting this failure's interval.
                reattach_attempts_     = 0;
                reattach_backoff_ms_   = 0;
                ESP_LOGW(TAG, "modem re-attached after %u attempt(s) — "
                              "config re-push queued",
                         (unsigned)reattach_attempts_);
                return;
            }

            if (modem_incompatible_)
            {
                // It answered — so it is responsive, just not compatible.  That
                // is the one failure a power cycle provably cannot fix, and
                // re-arming here would drop a 22 dBm module in and out every
                // 60 s for the rest of the flight while logging on every reply.
                if (cfg_.act_pin >= 0) gpio_set_level((gpio_num_t)cfg_.act_pin, 0);
                reattach_state_ = Reattach::Off;
                ESP_LOGE(TAG, "re-attach: modem answered with an incompatible "
                              "protocol version — radio permanently disabled");
                return;
            }

            if ((int32_t)(now_ms - reattach_deadline_ms_) >= 0)
            {
                // Unpower again between attempts so the next one is a real power
                // cycle rather than another poke at a stuck part.
                if (cfg_.act_pin >= 0) gpio_set_level((gpio_num_t)cfg_.act_pin, 0);
                armReattach(now_ms);
            }
            return;
        }
    }
}

void UartModemBackend::service()
{
    if (!began_)
    {
        serviceReattach(millis());
        return;
    }
    link_.poll(&UartModemBackend::onFrameTrampoline, this);

    // Deferred BOOT config re-push (see onFrame) — outside the poll handler.
    // DURABLE: the pending flag stays set until a push actually succeeds,
    // retrying on a timer. The modem announces BOOT exactly once, so this
    // re-push is the only mechanism keeping the pair on one modulation
    // after a daughterboard reboot/reflash — a single-shot push whose ack
    // frame is lost would leave the modem on boot defaults while it keeps
    // radiating and returning TX_RESULT ok=1: healthy-looking stats on a
    // dead link. send() holds TX while this is pending for the same reason.
    if (config_repush_pending_ && modem_alive_ &&
        (int32_t)(millis() - repush_retry_at_ms_) >= 0)
    {
        // 80 ms, NOT the 500 ms begin() uses.  pushConfig()'s ack wait is a
        // spin loop (poll + vTaskDelay(1)), so an unacked push burns the whole
        // budget inside service() — and 500 ms is 5x the OC's own
        // LOOP_STALL_THRESHOLD_US (out_computer/main/main.cpp: 100 ms), which is
        // why serviceLoRa() is wrapped in LOOP_STALL_INSTR at all.  begin() can
        // afford 500 ms because it runs once at boot; this runs on a flying
        // rocket's main loop and now also runs after a re-attach (#823), where
        // a modem that answers GET_IDENTITY but never acks SET_CONFIG would
        // otherwise stall the loop for 500 ms every 2 s indefinitely.
        // Reliability comes from the DURABLE 2 s retry below, not from the
        // length of any single wait: a modem mid radio-begin (>=100 ms of fixed
        // resets) simply acks a later attempt.
        if (pushConfig(cfg_freq_mhz_, cfg_sf_, cfg_bw_khz_, cfg_cr_,
                       cfg_tx_power_, /*start_rx=*/true,
                       /*ack_timeout_ms=*/80))
        {
            config_repush_pending_ = false;
            ESP_LOGI(TAG, "post-BOOT config re-push applied");
        }
        else
        {
            // Also the RF-dead recovery path: the modem retries a full
            // radio begin() on every SET_CONFIG, so keep knocking (slowly).
            repush_retry_at_ms_ = millis() + 2000;
            ESP_LOGW(TAG, "post-BOOT config re-push failed — retrying in 2 s");
        }
    }

    // Periodic link-health poll. STATUS carries the counters neither side
    // logs otherwise (modem-side uart_rx_crc_fails/resync_bytes, air-side
    // rx_crc_fail), and polling keeps the getStats() mirror fresh instead
    // of frozen at the last SET_CONFIG ack. ~8 B up / 60 B down every 2 s
    // is noise next to telemetry.
    if (modem_alive_ && (int32_t)(millis() - status_poll_at_ms_) >= 0)
    {
        status_poll_at_ms_ = millis() + STATUS_POLL_MS;
        (void)link_.sendFrame(MSG_GET_STATUS, nullptr, 0);
    }

    // Surface link degradation as it happens rather than on autopsy: log
    // when either side's UART error counters moved. Host side = frames the
    // modem sent us that arrived damaged; modem side = our frames damaged
    // in the other direction.
    if ((int32_t)(millis() - health_log_at_ms_) >= 0)
    {
        health_log_at_ms_ = millis() + HEALTH_LOG_MS;
        TR_UART_Link::Stats ls = {};
        link_.getStats(ls);
        const uint32_t modem_crc = last_status_.uart_rx_crc_fails;
        const uint32_t modem_rsn = last_status_.uart_rx_resync_bytes;
        if (ls.rx_crc_fails != health_host_crc_snap_ ||
            ls.rx_resync_bytes != health_host_resync_snap_ ||
            modem_crc != health_modem_crc_snap_ ||
            modem_rsn != health_modem_resync_snap_)
        {
            ESP_LOGW(TAG, "UART link health: host rx_crc=%u resync=%uB | "
                          "modem rx_crc=%u resync=%uB",
                     ls.rx_crc_fails, ls.rx_resync_bytes, modem_crc, modem_rsn);
            health_host_crc_snap_ = ls.rx_crc_fails;
            health_host_resync_snap_ = ls.rx_resync_bytes;
            health_modem_crc_snap_ = modem_crc;
            health_modem_resync_snap_ = modem_rsn;
        }
    }
}

bool UartModemBackend::send(const uint8_t* payload, size_t len)
{
    // config_repush_pending_: the modem just rebooted and is still on its
    // boot-default modulation — transmitting now would radiate on the wrong
    // channel/SF. Refuse (caller retries next loop) until the re-push lands.
    if (!modem_alive_ || config_repush_pending_ || payload == nullptr ||
        len == 0 || len > MAX_AIR_FRAME || tx_in_flight_)
    {
        return false;
    }

    uint8_t buf[sizeof(TxFrameHeader) + MAX_AIR_FRAME];
    tx_seq_++;
    buf[0] = tx_seq_;
    memcpy(buf + sizeof(TxFrameHeader), payload, len);
    if (!link_.sendFrame(MSG_TX_FRAME, buf, sizeof(TxFrameHeader) + len))
    {
        stats_.tx_fail++;
        return false;
    }
    tx_in_flight_ = true;
    tx_started_ms_ = millis();
    stats_.tx_started++;
    stats_.transmitting = true;
    return true;
}

bool UartModemBackend::canSend() const
{
    return modem_alive_ && !tx_in_flight_;
}

bool UartModemBackend::startReceive()
{
    if (!modem_alive_)
    {
        return false;
    }
    // The modem auto-listens whenever it isn't transmitting; this is a
    // belt-and-braces nudge, always considered successful.
    (void)link_.sendFrame(MSG_START_RX, nullptr, 0);
    return true;
}

bool UartModemBackend::readPacket(uint8_t* buf, size_t maxLen, size_t& len)
{
    len = 0;
    if (rxq_used_ == 0)
    {
        return false;
    }
    RxEntry& e = rx_queue_[rxq_head_];
    rxq_head_ = (rxq_head_ + 1) % RX_QUEUE_LEN;
    rxq_used_--;
    if (e.len > maxLen)
    {
        stats_.rx_len_drop++;
        return false;
    }
    memcpy(buf, e.data, e.len);
    len = e.len;
    stats_.last_rssi = e.rssi;
    stats_.last_snr = e.snr;
    return true;
}

bool UartModemBackend::waitTxIdle(uint32_t timeout_ms)
{
    const uint32_t deadline = millis() + timeout_ms;
    while (tx_in_flight_)
    {
        if (millis() >= deadline)
        {
            return false;
        }
        link_.poll(&UartModemBackend::onFrameTrampoline, this);
        serviceTxWatchdog();
        vTaskDelay(1);
    }
    return true;
}

bool UartModemBackend::reconfigure(float freq_mhz, uint8_t sf, float bw_khz,
                                   uint8_t cr, int8_t tx_power,
                                   bool wait_for_tx)
{
    if (!modem_alive_)
    {
        return false;
    }
    if (tx_in_flight_)
    {
        if (!wait_for_tx)
        {
            // Non-blocking contract (#405 rendezvous paths): busy → caller
            // retries next loop iteration.
            return false;
        }
        if (!waitTxIdle(2000))  // matches the direct driver's 2 s deadline
        {
            return false;
        }
    }
    return pushConfig(freq_mhz, sf, bw_khz, cr, tx_power, /*start_rx=*/true,
                      /*ack_timeout_ms=*/500);
}

bool UartModemBackend::hopToFrequencyMHz(float freq_mhz)
{
    if (!modem_alive_ || tx_in_flight_)
    {
        // Mirrors the direct driver: no retune mid-TX; caller keeps its
        // channel state and retries at the next packet boundary.
        return false;
    }
    HopFreqData d = {freq_mhz};
    if (!link_.sendFrame(MSG_HOP_FREQ, reinterpret_cast<uint8_t*>(&d),
                         sizeof(d)))
    {
        return false;
    }
    cfg_freq_mhz_ = freq_mhz;
    return true;
}

bool UartModemBackend::isInRxMode() const
{
    // Modem policy: always listening except while transmitting.
    return modem_alive_ && !tx_in_flight_;
}

void UartModemBackend::serviceTxWatchdog()
{
    if (tx_in_flight_ &&
        (millis() - tx_started_ms_) > TX_WATCHDOG_MS)
    {
        // TX_RESULT never came back (modem died mid-frame / UART glitch):
        // free the slot so canSend() can't wedge false forever (#105
        // philosophy). The modem's own watchdog guards its side.
        ESP_LOGW(TAG, "TX_RESULT watchdog fired (seq=%u)", tx_seq_);
        tx_in_flight_ = false;
        stats_.transmitting = false;
        stats_.tx_fail++;
        stats_.tx_watchdog_fires++;
    }
}

bool UartModemBackend::startScan(float start_mhz, float stop_mhz,
                                 uint16_t step_khz, uint16_t dwell_ms)
{
    if (!modem_alive_ || scan_active_ || tx_in_flight_ || step_khz == 0 ||
        stop_mhz <= start_mhz)
    {
        // Mirrors the direct driver's refusals (no scan mid-TX/mid-scan);
        // the caller retries.
        return false;
    }

    ScanRequestData d = {};
    d.start_mhz = start_mhz;
    d.stop_mhz = stop_mhz;
    d.step_khz = step_khz;
    d.dwell_ms = dwell_ms;
    if (!link_.sendFrame(MSG_START_SCAN, reinterpret_cast<uint8_t*>(&d),
                         sizeof(d)))
    {
        return false;
    }

    // The modem is silent when it refuses/drops a scan, so bound the wait:
    // sweep duration (steps x dwell, plus per-step retune overhead) doubled,
    // plus fixed slack for queueing + the result frame.
    uint32_t steps = (uint32_t)((stop_mhz - start_mhz) * 1000.0f / step_khz) + 1;
    if (steps > TR_LoRa_Comms::SCAN_MAX_SAMPLES)
    {
        steps = TR_LoRa_Comms::SCAN_MAX_SAMPLES;
    }
    scan_deadline_ms_ = millis() + 2 * steps * (dwell_ms + 5) + 3000;

    scan_active_ = true;
    scan_done_ = false;
    scan_count_ = 0;
    scan_start_mhz_ = start_mhz;
    scan_step_khz_ = step_khz;
    return true;
}

void UartModemBackend::serviceScan()
{
    // The sweep itself runs on the modem; frames drain via service(). Our
    // only job here is the never-wedge guarantee (IRadioLink contract):
    // a lost SCAN_RESULT must not gate the BS uplink forever (#379).
    if (scan_active_ && (int32_t)(millis() - scan_deadline_ms_) > 0)
    {
        ESP_LOGW(TAG, "scan timed out (SCAN_RESULT never arrived) — "
                      "reporting empty scan");
        scan_active_ = false;
        scan_done_ = true;
        scan_count_ = 0;
    }
}

void UartModemBackend::getStats(TR_LoRa_Comms::Stats& out) const
{
    out = stats_;
    out.enabled = modem_alive_;
    out.rx_mode = isInRxMode();
    out.transmitting = tx_in_flight_;
}
