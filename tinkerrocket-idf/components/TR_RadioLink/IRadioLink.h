#pragma once

#include <stddef.h>
#include <stdint.h>

#include <TR_LoRa_Comms.h>  // Stats struct is the shared stats currency

// Radio-backend seam for the out computer / base station (#410, tracking
// #408): the same application code drives either the direct SPI LLCC68
// (V7 boards, LoRaDirectBackend) or the UART radio daughterboard (V8,
// UartModemBackend). Selected by the board header's USE_UART_RADIO_MODEM.
//
// The surface below is EXACTLY the set of TR_LoRa_Comms methods the OC
// application calls (derived from a call-site audit of main.cpp) — no
// speculative additions. Contracts the backends must preserve:
//
// - send() returns true when the transmission STARTS; canSend() flips true
//   when airtime is finished. The hop machine uses canSend() as its
//   "TX done, safe to retune" signal, so a backend must never report
//   canSend()==true while its radio is still radiating (the UART modem
//   backend therefore allows only ONE in-flight frame even though the
//   link protocol supports a deeper credit window).
// - reconfigure(wait_for_tx=false) must be non-blocking: return false when
//   busy, the caller retries next loop iteration (#405 rendezvous paths).
//   wait_for_tx=true may block bounded (~2 s) and its result is a
//   synchronous decision (NVS persistence in uplink cmd 10/16).
// - hopToFrequencyMHz() is the lightweight per-packet retune: must return
//   false mid-TX (caller keeps channel state and retries at the next
//   packet boundary), never block.
// - readPacket()/canSend()/isInRxMode()/currentSpreadingFactor()/getStats()
//   answer locally (no blocking I/O in the query path).
class IRadioLink
{
public:
    virtual ~IRadioLink() = default;

    // Main-loop service step (completes TX, drains link I/O). Non-blocking.
    virtual void service() = 0;

    virtual bool send(const uint8_t* payload, size_t len) = 0;
    virtual bool canSend() const = 0;

    virtual bool startReceive() = 0;
    virtual bool readPacket(uint8_t* buf, size_t maxLen, size_t& len) = 0;

    virtual bool reconfigure(float freq_mhz, uint8_t sf, float bw_khz,
                             uint8_t cr, int8_t tx_power,
                             bool wait_for_tx = true) = 0;
    virtual bool hopToFrequencyMHz(float freq_mhz) = 0;

    virtual float currentFrequencyMHz() const = 0;
    virtual uint8_t currentSpreadingFactor() const = 0;
    virtual bool isInRxMode() const = 0;

    // Direct-backend housekeeping (missed-IRQ poll, stuck-TX watchdog).
    // The modem backend maps these to its own liveness checks / no-ops.
    virtual void pollDio1() = 0;
    virtual void serviceTxWatchdog() = 0;

    virtual void getStats(TR_LoRa_Comms::Stats& out) const = 0;

    // ---- Spectrum scan (pre-launch collision avoidance; BS caller, #414) --
    // Mirrors the TR_LoRa_Comms scan surface exactly (same call-site-audit
    // rule as above — this is the set the BS application uses). Contract:
    // startScan() kicks off a sweep (refused mid-TX or mid-scan); the caller
    // then calls serviceScan() every loop iteration until isScanDone(),
    // reads the samples, and consumeScanDone()s. A backend must NEVER leave
    // isScanDone() false forever after a startScan() that returned true —
    // the BS gates uplink TX on scan state (#379), so a wedged scan mutes
    // the uplink. On any failure path (modem reboot, lost SCAN_RESULT) the
    // backend reports done-with-zero-samples instead.
    // TR_LoRa_Comms::ScanSample is the shared sample currency.
    virtual bool startScan(float start_mhz, float stop_mhz, uint16_t step_khz,
                           uint16_t dwell_ms) = 0;
    virtual void serviceScan() = 0;
    virtual bool isScanDone() const = 0;
    virtual void consumeScanDone() = 0;
    virtual size_t getScanSampleCount() const = 0;
    virtual const TR_LoRa_Comms::ScanSample* getScanSamples() const = 0;
    virtual float getScanStartMHz() const = 0;
    virtual float getScanStepKHz() const = 0;
};
