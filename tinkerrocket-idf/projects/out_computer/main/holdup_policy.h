#pragma once

#include <stdint.h>

// #1166: is the hold-up supercap actually there?
//
// On rocket-computer-mini the +3V3 rail rides a 5 F supercap (C130) that a
// TPS61094 (U47) buck-charges at 100 mA to a 2.5 V termination — about two
// minutes from empty — and boosts back onto the rail when the pack bounces.
// The failure mode this watches for is silent by construction: with the cap
// never charged, +3V3 is fine, the board behaves normally, and the only
// symptom is that the bridge is not there when it is needed.  #999 found the
// charge-entry margin was inside the converter's tolerance band and fixed it
// in hardware (OSEL 3.0 V); this is the diagnostic that says so if a unit
// still ends up that way.  The out computer has the sense line
// (V_SCAP_ADC, board_m1.h), so the verdict is made here and travels to the
// app as the telemetry key "hu" — an advisory line, never an arm block.
//
// The rule: the cap counts as charged at or above `charged_v` (2.2 V of the
// 2.5 V termination, 77 % of the energy).  Under that bar the cap is
// CHARGING until it has been under the bar for `advisory_ms` (three minutes,
// against a ~two-minute ramp), then NOT_CHARGING.  The window is measured
// from boot on a cold start and from the LAST CHARGED reading otherwise, so
// a cap that a hold-up event just drained gets its recharge time before
// anyone is told about it.
//
// Pure: no IDF, no clock — the caller feeds uptime — so tests_cpp drives it.
namespace holdup_policy
{
    // Numerically identical to TR_BLE_To_APP::HoldupState, which is what the
    // value becomes on the wire; main.cpp static_asserts the two agree.
    enum State : uint8_t
    {
        NONE         = 0,   // nothing read (no sense line, ADC failed) — key absent
        CHARGING     = 1,   // under the bar, inside the grace window
        CHARGED      = 2,   // at or above the bar
        NOT_CHARGING = 3,   // under the bar for the whole grace window — advisory
        NO_READING   = 4,   // the sense exists but the ADC did not answer — advisory
    };

    inline const char* stateName(uint8_t s)
    {
        switch (s)
        {
            case CHARGING:     return "charging";
            case CHARGED:      return "charged";
            case NOT_CHARGING: return "NOT CHARGING";
            case NO_READING:   return "NO READING";
            default:           return "unknown";
        }
    }

    struct Tracker
    {
        uint8_t  state           = NONE;
        bool     seen_charged    = false;
        uint32_t last_charged_ms = 0;   // uptime of the last reading at/above the bar

        // One reading.  `scap_v` NaN = the sense is there but the ADC did not
        // answer: NO_READING, its own advisory — a dead sense must not be
        // silence either — and the grace clock is untouched.  `now_ms` is
        // uptime, wrap-safe: the elapsed arithmetic is unsigned.
        uint8_t update(float scap_v, uint32_t now_ms, float charged_v, uint32_t advisory_ms)
        {
            if (!(scap_v == scap_v))
            {
                state = NO_READING;
                return state;
            }
            if (scap_v >= charged_v)
            {
                seen_charged    = true;
                last_charged_ms = now_ms;
                state           = CHARGED;
                return state;
            }
            // Under the bar: how long has it been?  From boot (uptime 0) if the
            // cap has never read charged this session, else from the last time it did.
            const uint32_t since = seen_charged ? last_charged_ms : 0u;
            state = ((uint32_t)(now_ms - since) >= advisory_ms) ? NOT_CHARGING : CHARGING;
            return state;
        }
    };

    // Console trace cadence for the first-article cold-start log #1166 asks
    // for: a line whenever V_SCAP has moved `delta_v` since the last line, or
    // at least every `period_ms` when it is steady — plus the first sample and
    // every change of verdict.  During the 100 mA ramp (~20 mV/s into 5 F)
    // 50 mV is a line every ~2.5 s, the charge profile the first article
    // needs; terminated, one a minute; a discharge (~84 mV/s riding the cap)
    // logs every second, which is the hold-time curve.  NaN-to-NaN is steady
    // and falls to the period rule, so a dead sense does not spam.
    inline bool traceDue(bool state_changed, bool ever_traced,
                         float scap_v, float last_traced_v,
                         uint32_t now_ms, uint32_t last_trace_ms,
                         float delta_v, uint32_t period_ms)
    {
        if (state_changed || !ever_traced) return true;
        const bool v_nan = !(scap_v == scap_v);
        const bool l_nan = !(last_traced_v == last_traced_v);
        if (v_nan != l_nan) return true;
        if (!v_nan)
        {
            const float d = scap_v - last_traced_v;
            if (d >= delta_v || d <= -delta_v) return true;
        }
        return (uint32_t)(now_ms - last_trace_ms) >= period_ms;
    }
}
