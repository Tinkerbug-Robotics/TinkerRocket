#pragma once

#include <compat.h>
// RadioLib (vendored) trips -Woverloaded-virtual: its module classes hide
// PhysicalLayer base methods by design. Suppress it just for this include so
// every consumer of this header (main.cpp, TR_LoRa_Comms.cpp) stays
// warning-clean without patching upstream sources (#88). RadioLib's own .cpp
// files are handled by the same suppression in components/RadioLib/CMakeLists.txt.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <RadioLib.h>
#pragma GCC diagnostic pop
#include "EspHal.h"

class TR_LoRa_Comms
{
public:
    struct Config
    {
        bool enabled = false;
        int cs_pin = -1;
        int dio1_pin = -1;
        int rst_pin = -1;
        int busy_pin = -1;
        // RF-switch RX enable for modules with a discrete antenna switch
        // (TX side is DIO2, see setDio2AsRfSwitch in begin()). RadioLib
        // drives it HIGH in RX / LOW in TX+idle via setRfSwitchPins.
        // -1 = not wired (existing V7 OC / original-BS boards).
        int rxen_pin = -1;
        int spi_sck = -1;
        int spi_miso = -1;
        int spi_mosi = -1;
        spi_host_device_t spi_host = SPI2_HOST;
        float freq_mhz = 915.0f;
        // Defaults are the fleet operating point, and a legal LLCC68 pair
        // (SF10/BW125 is not: LLCC68 caps SF at 9 for BW125).
        uint8_t spreading_factor = 8;
        float bandwidth_khz = 250.0f;
        uint8_t coding_rate = 7;
        uint16_t preamble_len = 16;
        int8_t tx_power_dbm = 20;
        bool crc_on = true;
        bool rx_boosted_gain = true;
        bool syncword_private = true;
    };

    struct Stats
    {
        bool enabled = false;
        bool transmitting = false;
        bool rx_mode = false;
        uint32_t tx_started = 0;
        uint32_t tx_ok = 0;
        uint32_t tx_fail = 0;
        uint32_t rx_count = 0;
        uint32_t rx_crc_fail = 0;
        uint32_t rx_len_drop = 0;  // #288: RX dropped for bad length (0 or > maxLen)
        // #520: rx_done_ was latched but the radio had no packet — a stale DIO1
        // level re-armed the flag for a packet already consumed. Caught and
        // discarded instead of re-reading the SX126x buffer (which produced
        // byte-identical duplicate packets). Nonzero here is EXPECTED and healthy:
        // it means the race is still happening and is being caught.
        uint32_t rx_spurious = 0;
        uint32_t isr_count = 0;
        uint32_t tx_watchdog_fires = 0;  // tx_ongoing_ force-cleared by watchdog (#105)
        float last_rssi = 0.0f;
        float last_snr = 0.0f;
        int16_t last_error = RADIOLIB_ERR_NONE;
    };

    TR_LoRa_Comms();
    ~TR_LoRa_Comms() = default;

    bool begin(const Config& cfg, bool debug = false);
    void service();
    bool send(const uint8_t* payload, size_t len);
    bool canSend() const;
    bool isEnabled() const { return enabled_; }
    bool isInRxMode() const { return rx_mode_; }
    void getStats(Stats& out) const;

    // RX support
    bool startReceive();
    bool readPacket(uint8_t* buf, size_t maxLen, size_t& len);

    // Poll DIO1 pin directly (fallback if hardware interrupt doesn't fire)
    void pollDio1();

    // TX watchdog (#105 follow-up).  Force-clears tx_ongoing_ if it has
    // been stuck for longer than any plausible TX could take.  Without
    // this, a missed DIO1 IRQ + a misclassified pollDio1 (rx_mode_ true
    // at the moment of polling) wedges the radio: hopToFrequencyMHz and
    // reconfigure both refuse to run while tx_ongoing_=true, so the
    // entire link locks up until the device reboots.  Field-confirmed
    // failure mode where reconfigure hits its 2 s wait deadline.
    //
    // Designed to be called every main-loop iteration (alongside
    // pollDio1).  Cheap when not stuck (one millis() compare).
    void serviceTxWatchdog();

    // Runtime reconfiguration (LLCC68: must set BW before SF).
    // wait_for_tx (#398): when true (default), blocks up to 2 s for an
    // in-flight TX to finish before retuning — needed on user-initiated
    // config paths where the caller won't retry.  Pass FALSE from main-loop
    // service paths (rendezvous enter/exit): a packet's airtime at SF8/BW250
    // is 100-200 ms and the spin-wait stalled loop_oc for all of it (bench:
    // "wait-for-TX took 115-215 ms"); a busy return lets the caller simply
    // retry next iteration, mirroring hopToFrequencyMHz's can't-retune-mid-TX
    // contract.
    bool reconfigure(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr, int8_t tx_power,
                     bool wait_for_tx = true);

    // Apply the frame-format parameters that begin() fixes (preamble length,
    // CRC, RX boosted gain, sync word). These were historically "boot-fixed"
    // only because this class exposed no setter — the underlying SX126x
    // commands are runtime-settable. Needed by the UART radio modem (#409):
    // its SET_CONFIG carries all eight radio fields and must be fully
    // authoritative, or a host whose preamble differs from the modem's boot
    // default silently gets the wrong airtime per packet (the FCC dwell
    // budget in RocketComputerTypes.h is computed from the host's value).
    // Call only while the radio is idle (canSend() && !isScanActive());
    // returns false otherwise. Like reconfigure(), exits RX mode — the
    // caller re-enters RX (startReceive) afterwards.
    bool applyFrameParams(uint16_t preamble_len, bool crc_on,
                          bool rx_boosted_gain, bool syncword_private);

    // Lightweight frequency-only retune for per-packet hopping (#40 / #41).
    // Unlike reconfigure(), this does not touch BW/SF/CR/power — those
    // remain locked across hops within a session.  Returns false if the
    // radio is mid-TX, mid-scan, or the underlying setFrequency call fails;
    // in those cases the caller should keep its current channel state and
    // try again on the next packet boundary.  If the radio was in RX mode,
    // it re-enters RX after the retune so the next packet is received on
    // the new frequency without an extra startReceive() call from the
    // caller.
    bool hopToFrequencyMHz(float freq_mhz);

    // Reports the radio's currently-tuned frequency in MHz (the last value
    // accepted by reconfigure() or hopToFrequencyMHz()).  Useful for the
    // hop state machine to detect whether a retune is actually required.
    float currentFrequencyMHz() const { return cfg_freq_mhz_; }

    // Reports the radio's currently-configured spreading factor.  The
    // application uses this to compute an SF-aware SNR floor for RX
    // (#90 follow-up): rendezvous mode and operating mode can run at
    // different SFs, so the threshold needs to follow the radio.
    uint8_t currentSpreadingFactor() const { return cfg_sf_; }

    // ---- Spectrum scan (pre-launch collision avoidance) ------------------
    // Non-blocking channel-hopping RSSI scan. Step through a frequency range,
    // dwell in RX mode for `dwell_ms` per channel, then read instantaneous
    // RSSI.  Call serviceScan() from the main loop each iteration until
    // isScanDone() returns true.  While active, normal RX is suspended; the
    // previous frequency is restored on completion.
    static constexpr size_t SCAN_MAX_SAMPLES = 128;
    struct ScanSample
    {
        float  freq_mhz;
        int8_t rssi_dbm;   // clamped to int8 (−128..0 dBm is well outside useful range)
    };

    bool startScan(float start_mhz, float stop_mhz, uint16_t step_khz, uint16_t dwell_ms);
    void serviceScan();
    bool isScanActive() const { return scan_state_ != ScanState::Idle && scan_state_ != ScanState::Done; }
    bool isScanDone() const   { return scan_state_ == ScanState::Done; }
    void consumeScanDone()    { scan_state_ = ScanState::Idle; }
    size_t getScanSampleCount() const { return scan_count_; }
    const ScanSample* getScanSamples() const { return scan_samples_; }
    float getScanStartMHz() const { return scan_start_mhz_; }
    float getScanStepKHz()  const { return scan_step_khz_; }

private:
    static void IRAM_ATTR onDio1ISR();

    static TR_LoRa_Comms* instance_;

    bool enabled_ = false;
    bool debug_ = false;
    volatile bool tx_done_ = false;
    volatile bool rx_done_ = false;
    bool tx_ongoing_ = false;
    bool rx_mode_ = false;
    volatile uint32_t isr_count_ = 0;
    int dio1_pin_ = -1;

    // TX watchdog state (#105 follow-up).  tx_started_ms_ is set in
    // send() and cleared whenever tx_ongoing_ becomes false.  The
    // watchdog (serviceTxWatchdog) compares (now - tx_started_ms_)
    // against TX_WATCHDOG_MS to detect a stuck flag.
    uint32_t tx_started_ms_ = 0;
    static constexpr uint32_t TX_WATCHDOG_MS = 3000;  // generous: SF12 BW125 ToA ~2.5s

    // Last-known-good radio config (for rollback on reconfigure failure)
    float   cfg_freq_mhz_ = 915.0f;
    uint8_t cfg_sf_ = 8;
    float   cfg_bw_khz_ = 250.0f;
    uint8_t cfg_cr_ = 7;
    int8_t  cfg_tx_power_ = 20;

    EspHal* hal_ = nullptr;
    Module* module_ = nullptr;
    LLCC68* radio_ = nullptr;
    Stats stats_ = {};

    // ---- Scan state ------------------------------------------------------
    enum class ScanState : uint8_t { Idle, SetFreq, Dwell, Done };
    // #567: single exit point for the scan state machine — restore the
    // operating frequency, re-enter RX, mark Done. EVERY path out of a scan
    // must go through this, because isScanActive() gates send()/canSend()/
    // hopToFrequencyMHz(): a scan that never reaches Done kills TX and
    // hopping until reboot.
    void finishScan(const char* why);
    ScanState scan_state_    = ScanState::Idle;
    uint16_t  scan_idx_      = 0;
    uint16_t  scan_n_steps_  = 0;
    uint16_t  scan_dwell_ms_ = 30;
    uint32_t  scan_dwell_start_ms_ = 0;
    // #567: wall-clock backstop for the whole scan (defense in depth — the
    // state machine terminates by construction, this catches regressions and
    // unknown radio failure modes). Sized in startScan() well above the
    // slowest plausible healthy scan so it never false-fires.
    uint32_t  scan_deadline_ms_ = 0;
    float     scan_start_mhz_ = 0.0f;
    float     scan_step_khz_  = 0.0f;
    size_t    scan_count_    = 0;
    ScanSample scan_samples_[SCAN_MAX_SAMPLES] = {};
};
