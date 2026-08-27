#pragma once

#include <RadioModemProtocol.h>
#include <TR_UART_Link.h>

#include "IRadioLink.h"

// UART radio-daughterboard backend (V8 boards, #410): implements the
// IRadioLink contract over the transparent-modem protocol (#409,
// RadioModemProtocol.h). The daughterboard owns the LLCC68; this backend
// tunnels opaque air frames and mirrors radio state locally so every
// query-path call answers without link I/O.
//
// Contract-preserving choices (see IRadioLink.h):
// - ONE in-flight TX at a time: canSend() == no outstanding TX_RESULT.
//   The protocol's 8-credit window is deliberately not used — the hop
//   machine relies on canSend() meaning "airtime finished".
// - reconfigure(): SET_CONFIG → bounded wait for the STATUS ack (the
//   modem always echoes STATUS after SET_CONFIG). wait_for_tx=true first
//   waits (bounded) for an outstanding TX; =false returns false when busy,
//   matching the direct driver's non-blocking rendezvous contract (#405).
// - hopToFrequencyMHz(): refuses mid-TX locally (mirrors the driver's
//   can't-retune-mid-TX rule), otherwise fire-and-forget HOP_FREQ.
// - A modem BOOT message (daughterboard rebooted/power-cycled) clears the
//   in-flight slot and triggers an automatic config re-push.
#include "modem_rail_sequence.h"

class UartModemBackend final : public IRadioLink
{
public:
    struct Config
    {
        TR_UART_Link::Config uart;
        int act_pin = -1;  // daughterboard power gate (LORA_ACT); -1 = none
        // Boot-fixed radio fields forwarded in every SET_CONFIG
        uint16_t preamble_len = 12;
        bool crc_on = true;
        bool rx_boosted_gain = true;
        bool syncword_private = true;
    };

    // Powers the daughterboard (act_pin high), opens the UART, waits
    // (bounded) for the modem's BOOT identity, verifies the protocol
    // version, then pushes the initial radio config. Returns false on
    // no/incompatible modem — the OC then runs radio-less, mirroring a
    // failed TR_LoRa_Comms::begin().
    bool begin(const Config& cfg, float freq_mhz, uint8_t sf, float bw_khz,
               uint8_t cr, int8_t tx_power, bool debug);

    void service() override;

    bool send(const uint8_t* payload, size_t len) override;
    bool canSend() const override;

    bool startReceive() override;
    bool readPacket(uint8_t* buf, size_t maxLen, size_t& len) override;

    bool reconfigure(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr,
                     int8_t tx_power, bool wait_for_tx = true) override;
    bool hopToFrequencyMHz(float freq_mhz) override;

    float currentFrequencyMHz() const override { return cfg_freq_mhz_; }
    uint8_t currentSpreadingFactor() const override { return cfg_sf_; }
    bool isInRxMode() const override;

    // No DIO1 on this side of the link; the watchdog slot instead clears a
    // TX_RESULT that never arrived (modem died mid-frame) so canSend()
    // can't wedge false forever — same failure philosophy as #105.
    void pollDio1() override {}
    void serviceTxWatchdog() override;

    void getStats(TR_LoRa_Comms::Stats& out) const override;

    // Spectrum scan (#414): START_SCAN is fire-and-forget on the wire —
    // the modem runs the sweep autonomously and answers with one
    // SCAN_RESULT frame, which we cache so every query answers locally.
    // The modem is silent on a refused/dropped scan, so a host-side
    // deadline (sweep duration + margin) fails the scan as
    // done-with-zero-samples; a modem BOOT mid-scan does the same.
    bool startScan(float start_mhz, float stop_mhz, uint16_t step_khz,
                   uint16_t dwell_ms) override;
    void serviceScan() override;
    bool isScanDone() const override { return scan_done_; }
    void consumeScanDone() override { scan_done_ = false; }
    size_t getScanSampleCount() const override { return scan_count_; }
    const TR_LoRa_Comms::ScanSample* getScanSamples() const override
    {
        return scan_samples_;
    }
    float getScanStartMHz() const override { return scan_start_mhz_; }
    float getScanStepKHz() const override { return scan_step_khz_; }

    // Modem-side identity captured from BOOT/IDENTITY (fw version, chip,
    // capabilities) — surfaced for status/logging.
    const radio_modem::ModemIdentityData& identity() const { return identity_; }
    bool modemAlive() const { return modem_alive_; }

private:
    static void onFrameTrampoline(void* ctx, uint8_t type,
                                  const uint8_t* payload, size_t len);
    void onFrame(uint8_t type, const uint8_t* payload, size_t len);
    bool pushConfig(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr,
                    int8_t tx_power, bool start_rx, uint32_t ack_timeout_ms);
    bool waitTxIdle(uint32_t timeout_ms);

    TR_UART_Link link_;
    Config cfg_{};
    bool began_ = false;
    bool debug_ = false;
    bool modem_alive_ = false;

    // In-flight TX slot (single — see header comment)
    bool tx_in_flight_ = false;
    uint8_t tx_seq_ = 0;
    uint32_t tx_started_ms_ = 0;
    static constexpr uint32_t TX_WATCHDOG_MS = 3000;  // matches TR_LoRa_Comms

    // Cached radio params (last accepted by begin/reconfigure/hop)
    float cfg_freq_mhz_ = 915.0f;
    uint8_t cfg_sf_ = 10;
    float cfg_bw_khz_ = 125.0f;
    uint8_t cfg_cr_ = 7;
    int8_t cfg_tx_power_ = 20;

    // RX queue: LoRa airtime paces arrivals (~10 Hz), the OC polls every
    // loop iteration — 4 slots is generous.
    static constexpr size_t RX_QUEUE_LEN = 4;
    struct RxEntry
    {
        uint8_t len;
        float rssi;
        float snr;
        uint8_t data[radio_modem::MAX_AIR_FRAME];
    };
    RxEntry rx_queue_[RX_QUEUE_LEN] = {};
    uint8_t rxq_head_ = 0;
    uint8_t rxq_used_ = 0;

    // STATUS ack tracking for reconfigure()
    uint32_t status_rx_count_ = 0;

    // BOOT config re-push deferred out of the frame handler (see onFrame).
    // Durable: stays set until a pushConfig() succeeds; send() refuses while
    // pending (the modem is on boot defaults until the push lands).
    bool config_repush_pending_ = false;
    uint32_t repush_retry_at_ms_ = 0;

    // Re-attach after a failed begin() — the "later retry" begin()'s failClosed()
    // comment has always promised and nothing ever performed.  begin() runs once
    // per boot on both hosts, so before this a daughterboard that was absent,
    // unpowered or wedged at boot stayed dark for the entire session: act_pin
    // low so it could never announce BOOT, and began_ false so service()
    // returned before polling and could never hear it if it did.  The hot-join
    // path in onFrame() was therefore unreachable.
    //
    // Runs from service() while !began_, so no host changes and both MCUs get
    // it.  Strictly non-blocking: begin()'s wait loop blocks 2.5-4 s, which is
    // survivable once at boot but not on a flying rocket's main loop, so the
    // probe is spread across service() calls instead.
    //
    // A successful re-attach hands off to config_repush_pending_ above rather
    // than pushing config itself — that path is already durable and already
    // covers the RF-dead modem.
    enum class Reattach : uint8_t
    {
        Off,       // never failed, already re-attached, or failed permanently
        Backoff,   // module unpowered, waiting out the interval
        Probing,   // module powered, UART open, listening for BOOT/IDENTITY
    };
    Reattach reattach_state_ = Reattach::Off;
    uint32_t reattach_at_ms_ = 0;        // when Backoff expires
    uint32_t reattach_deadline_ms_ = 0;  // when the current probe gives up
    uint32_t reattach_identity_at_ms_ = 0;
    uint32_t reattach_backoff_ms_ = 0;   // current interval, doubles to the cap
    uint16_t reattach_attempts_ = 0;
    // TR_UART_Link has no end()/isOpen(), so re-calling begin() on an open port
    // would re-install the driver.  Track it here instead.
    bool link_open_ = false;

    // #700: the host must not drive the daughterboard's UART while its rail
    // is gated off.  TR_UART_Link has no end(), so the driver stays installed
    // for the life of the boot — parking is done at the GPIO matrix instead,
    // exactly as the FC does for the RunCam.  Ordering is the whole point:
    // park BEFORE dropping act_pin, attach AFTER raising it.
    void parkModemUart();
    void attachModemUart();
    void runRailStep(modem_rail::Step s, int act_pin);
    void railDown(int act_pin);
    void railUp(int act_pin);
    // Set by onFrame() when a modem ANSWERS with the wrong protocol version.
    // The probe cannot otherwise tell "incompatible" from "silent", and only
    // the silent case is worth power-cycling.
    bool modem_incompatible_ = false;
    // Set only by an IDENTITY frame, which is a REPLY to our GET_IDENTITY and
    // therefore proves both directions of the link.  An unsolicited BOOT proves
    // only that the modem transmits.
    bool identity_reply_seen_ = false;

    // First retry is soon because the overwhelmingly common cause is a
    // daughterboard that was simply not plugged in or not yet powered; the cap
    // keeps a permanently dead module from power-cycling itself forever.
    static constexpr uint32_t REATTACH_FIRST_MS = 5000;
    static constexpr uint32_t REATTACH_MAX_MS   = 60000;

    void serviceReattach(uint32_t now_ms);
    void armReattach(uint32_t now_ms);

    // Periodic link-health poll + change-triggered logging (see service())
    static constexpr uint32_t STATUS_POLL_MS = 2000;
    static constexpr uint32_t HEALTH_LOG_MS = 5000;
    uint32_t status_poll_at_ms_ = 0;
    uint32_t health_log_at_ms_ = 0;
    uint32_t health_host_crc_snap_ = 0;
    uint32_t health_host_resync_snap_ = 0;
    uint32_t health_modem_crc_snap_ = 0;
    uint32_t health_modem_resync_snap_ = 0;

    // Scan state (cached SCAN_RESULT; see startScan comment)
    bool scan_active_ = false;
    bool scan_done_ = false;
    uint32_t scan_deadline_ms_ = 0;
    size_t scan_count_ = 0;
    float scan_start_mhz_ = 0.0f;
    float scan_step_khz_ = 0.0f;
    TR_LoRa_Comms::ScanSample scan_samples_[TR_LoRa_Comms::SCAN_MAX_SAMPLES] = {};

    // Stats mirror in TR_LoRa_Comms currency (fed by TX_RESULT / RX_FRAME /
    // STATUS)
    TR_LoRa_Comms::Stats stats_ = {};
    radio_modem::ModemIdentityData identity_ = {};
    radio_modem::ModemStatusData last_status_ = {};
};
