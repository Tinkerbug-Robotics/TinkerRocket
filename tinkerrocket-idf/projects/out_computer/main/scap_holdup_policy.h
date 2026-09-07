#pragma once

#include <stdint.h>
#include <math.h>

// #1166: is the +3V3 hold-up capacitor actually charged?
//
// The TPS61094 (U47) buck-charges the supercap (C130, 5 F) at 100 mA to a
// 2.5 V termination and boosts from it when the pack goes away. The failure
// this file exists for is silent by construction: a converter that never
// enters its charging state (#999 was one way to get there — a bypass-entry
// margin inside the part's tolerance band) leaves +3V3 fine, VBUCK_OK high
// and the board behaving normally, right up until the pack bounces and the
// bridge is not there. Nothing on the board reports it. V_SCAP_ADC — a
// 100 k / 100 k divider into OC GPIO8 — is the only observable, so the OC
// samples it once a second from boot, logs the charge ramp, and turns
// "still flat minutes after power-on" into an advisory the operator can see.
//
// Pure so the verdict table and the log cadence are host-testable
// (rail_restore_policy.h pattern). The ADC read itself lives in main.cpp.
//
// Clock: OC uptime. The OC boots when the pack (or USB) is connected, which
// is also when V_BUCK comes up and charging starts — with the flight-computer
// rail still OFF, so there is no NAND to log to; the console line and the
// BLE key are the record. An OC reboot with a charged cap restarts the
// window but reads CHARGED on its first sample, so nothing false is raised.

namespace ScapHoldupPolicy {

// Wire values for the BLE telemetry "hup" key, mirrored in both apps
// (TelemetryData.holdupState). 0 is reserved for "not reported" so that a
// zero-initialised TelemetryData — every builder does `= {}` — emits nothing
// on the wire, the #850 lesson.
enum HoldupState : uint8_t {
    HU_NOT_REPORTED = 0,  // no V_SCAP sense on this board (V7/V8/V9 OC), or not sampled yet
    HU_CHARGING     = 1,  // below the charged threshold, charge window still open
    HU_CHARGED      = 2,  // reached the charged threshold (latched through the hysteresis band)
    HU_LOW          = 3,  // below threshold with the window spent — never charged, or fell
    HU_NO_READING   = 4,  // the sense exists but the ADC did not answer
};
static_assert(HU_NOT_REPORTED == 0, "0 must stay 'absent' on the wire");

// Rise threshold. Termination is 2.5 V (VCHG, R135); 2.2 V is ~88% of the
// stored-energy target and sits above every realistic charging state of a
// healthy part, while a converter stuck in bypass never gets near it.
inline constexpr float kChargedV = 2.20f;
// Fall threshold: CHARGED drops to LOW only below this, 100 mV under the
// rise, so a cap parked near the threshold cannot flicker the advisory.
inline constexpr float kLowV = 2.10f;

// How long after OC boot "still below kChargedV" becomes a verdict.
//
// Nominal: 5 F at 100 mA from empty reaches 2.2 V at 110 s and terminates at
// 125 s. Corner: EDLC capacitance runs to +30 % and the ICHG code is about
// -10 %, so 6.5 F at 90 mA crosses 2.2 V at ~159 s. 180 s leaves ~20 s at
// that corner; a fatter or slower cap than that clears itself the moment it
// crosses, since CHARGED is evaluated on every sample regardless of the
// window. (A pack bounce that half-drains a charged cap recharges in under a
// minute and is reported as a real LOW while it does.)
inline constexpr uint32_t kWindowMs = 180000;

// Console log cadence: a line whenever V_SCAP has moved this much since the
// last line, or at least this often when it is steady. During the 100 mA ramp
// (~20 mV/s into 5 F) that is a line every ~2.5 s — the charge profile the
// first article needs — and one a minute once terminated. A discharge on the
// cap (~84 mV/s) logs every second, which is the hold-time curve.
inline constexpr float    kLogDeltaV   = 0.05f;
inline constexpr uint32_t kLogPeriodMs = 60000;

inline const char* name(HoldupState s)
{
    switch (s) {
        case HU_CHARGING:   return "CHARGING";
        case HU_CHARGED:    return "CHARGED";
        case HU_LOW:        return "LOW";
        case HU_NO_READING: return "NO READING";
        default:            return "n/a";
    }
}

// The verdict for one sample. NaN means the ADC did not answer.
inline HoldupState next(HoldupState prev, float v_scap, uint32_t uptime_ms)
{
    if (!(v_scap == v_scap)) return HU_NO_READING;
    if (v_scap >= kChargedV) return HU_CHARGED;
    if (prev == HU_CHARGED) {
        // Inside the hysteresis band the latch holds; below it the cap has
        // genuinely lost charge, window or no window.
        return (v_scap >= kLowV) ? HU_CHARGED : HU_LOW;
    }
    if (prev == HU_LOW) return HU_LOW;   // only a real crossing of kChargedV clears it
    return (uptime_ms >= kWindowMs) ? HU_LOW : HU_CHARGING;
}

// Whether this sample earns a console line. `have_logged` false = nothing
// printed yet this boot (always log the first sample). NaN-to-NaN is steady
// and falls to the period rule, so a dead ADC does not spam once a second.
inline bool shouldLog(bool state_changed, bool have_logged,
                      float v_scap, float last_logged_v,
                      uint32_t now_ms, uint32_t last_log_ms)
{
    if (state_changed || !have_logged) return true;
    const bool v_nan = !(v_scap == v_scap);
    const bool l_nan = !(last_logged_v == last_logged_v);
    if (v_nan != l_nan) return true;
    if (!v_nan && fabsf(v_scap - last_logged_v) >= kLogDeltaV) return true;
    return (uint32_t)(now_ms - last_log_ms) >= kLogPeriodMs;
}

}  // namespace ScapHoldupPolicy
