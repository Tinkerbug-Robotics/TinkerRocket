#ifndef IIS2MDC_POLL_GATE_H
#define IIS2MDC_POLL_GATE_H

#include <stdint.h>

// Attempt gate for the IIS2MDC magnetometer poll (#1111).
//
// The IIS2MDC has no DRDY line in the poll path (FC: INT not wired into the
// collector; mini: INT not wired at all), so pollIMUdata — which wakes on
// every ISM6 DRDY at 1920-3840 Hz — throttles the blocking I2C read to the
// chip's 100 Hz ODR through this gate.  Header-only and IDF-free so the
// schedule is host-testable (tests_cpp/test_iis2mdc_poll_gate.cpp).
//
// Rules:
//   * markAttempt() advances the clock on EVERY attempt, success or failure.
//     The gate this replaces advanced only on success, so a chip that stopped
//     answering was retried on every DRDY wake and each retry's blocking time
//     was paid straight out of the IMU sample rate.
//   * STALL_FAILS consecutive failures declare a stall.  While stalled the
//     caller probes (WHO_AM_I + re-configure) instead of reading, and the
//     probe cadence backs off from STALL_RETRY_MIN_US, doubling per failed
//     probe up to STALL_RETRY_MAX_US.  A chip that comes back is noticed
//     within one probe period; a chip that is gone costs the IMU loop one
//     bounded transaction per probe period, not one per DRDY.
//   * A successful probe clears the stall; the next data read is due one
//     PERIOD_US later.
//
// Timestamps are the 32-bit microsecond clock (esp_timer / micros()), which
// wraps every ~71.6 min; every comparison is a signed difference, so the wrap
// is harmless as long as reset(now) seeds the clock.  Seeding with 0 is NOT
// safe: begin() can run long after boot (the mini brings the sensor rail up
// on command), and a (now - 0) difference past 2^31 us reads negative — the
// gate would then stay shut until the clock wrapped.
struct Iis2mdcPollGate
{
    static constexpr uint32_t PERIOD_US          = 10000u;     // 100 Hz, the ODR
    static constexpr uint32_t STALL_FAILS        = 5u;         // 50 ms of silence at 100 Hz
    static constexpr uint32_t STALL_RETRY_MIN_US = 1000000u;   // first probe 1 s in ...
    static constexpr uint32_t STALL_RETRY_MAX_US = 32000000u;  // ... backing off to 32 s

    enum Event : uint8_t { EV_NONE = 0, EV_STALLED, EV_RECOVERED };

    uint32_t last_attempt_us = 0;
    uint32_t consec_fails    = 0;
    bool     stalled         = false;
    uint32_t stall_retry_us  = STALL_RETRY_MIN_US;

    // Lifetime counters, for the console diag.
    uint32_t read_ok      = 0;
    uint32_t read_fail    = 0;
    uint32_t stall_events = 0;
    uint32_t recoveries   = 0;

    // Seed the clock so the first attempt is due immediately.
    void reset(uint32_t now_us)
    {
        *this = Iis2mdcPollGate{};
        last_attempt_us = now_us - PERIOD_US;
    }

    uint32_t periodUs() const { return stalled ? stall_retry_us : PERIOD_US; }

    bool due(uint32_t now_us) const
    {
        return (int32_t)(now_us - last_attempt_us) >= (int32_t)periodUs();
    }

    // Call before every transfer, whatever its outcome.
    void markAttempt(uint32_t now_us) { last_attempt_us = now_us; }

    // Report the attempt's outcome.  Returns the state transition, if any.
    Event onResult(bool ok)
    {
        if (ok)
        {
            read_ok++;
            consec_fails = 0;
            if (stalled)
            {
                stalled = false;
                stall_retry_us = STALL_RETRY_MIN_US;
                recoveries++;
                return EV_RECOVERED;
            }
            return EV_NONE;
        }

        read_fail++;
        if (consec_fails < UINT32_MAX) consec_fails++;
        if (!stalled)
        {
            if (consec_fails >= STALL_FAILS)
            {
                stalled = true;
                stall_retry_us = STALL_RETRY_MIN_US;
                stall_events++;
                return EV_STALLED;
            }
            return EV_NONE;
        }
        // A failed probe: back off.
        stall_retry_us = (stall_retry_us < STALL_RETRY_MAX_US / 2u) ? stall_retry_us * 2u
                                                                     : STALL_RETRY_MAX_US;
        return EV_NONE;
    }
};

#endif
