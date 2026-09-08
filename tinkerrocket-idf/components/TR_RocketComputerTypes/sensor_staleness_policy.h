#pragma once

// #1137 items 10 and 12 — when the sensor-health scorecard should call a
// sensor stale.
//
// The scorecard (#303) is the operator's pre-launch go/no-go, and its baro and
// IMU entries carried comments saying they mirrored the flight loop's
// baro_healthy / ism6_fresh predicates.  They did not.  The flight predicates
// are binary and treat "present but not answering" as dead; the scorecard
// mapped it to SH_DEGRADED, which the iOS readiness rollup does not count as a
// hard fault (TelemetryData.swift builds hardFault from `.bad` alone).  So a
// sensor the FC itself had already stopped trusting -- one whose staleness
// disables the #258 accel-only launch fallback and burnout detection --
// showed the operator amber "caution" and never said "do not fly".
//
// The GNSS entry had no freshness test at all.  have_gnss_si latches on the
// first sample and gnss_latest_si keeps its last value forever, so a receiver
// that died after acquiring one 3D fix reported SH_OK indefinitely, on the app
// and on the base-station relay.
//
// Raw freshness is too twitchy to drive a red light directly: these sensors
// are polled, a single missed sample is normal, and a verdict that flickers
// between green and red is one the operator learns to ignore.  So the raw
// predicate is debounced symmetrically -- it must hold for kAssertMs before
// the scorecard believes it, and clear for kHoldMs before the scorecard
// forgives it.  The hold is the longer of the two on purpose: a sensor that
// drops out repeatedly should read bad continuously rather than strobing.

#include <stdint.h>

namespace sensor_staleness
{

// Continuous staleness required before the verdict drops.
inline constexpr uint32_t kAssertMs = 2000;

// Continuous freshness required before it recovers.
inline constexpr uint32_t kHoldMs = 5000;

struct State
{
    uint32_t stale_since_ms = 0;
    uint32_t fresh_since_ms = 0;
    bool     stale_pending  = false;
    bool     fresh_pending  = false;
    bool     asserted       = false;
};

/// `fresh_now` is the raw per-tick freshness test (age < timeout).  Returns
/// true when the scorecard should treat the sensor as stale.
inline bool step(State &s, bool fresh_now, uint32_t now_ms)
{
    if (!fresh_now)
    {
        s.fresh_pending = false;
        if (!s.stale_pending)
        {
            s.stale_pending  = true;
            s.stale_since_ms = now_ms;
        }
        if (!s.asserted &&
            (uint32_t)(now_ms - s.stale_since_ms) >= kAssertMs)
        {
            s.asserted = true;
        }
    }
    else
    {
        s.stale_pending = false;
        if (s.asserted)
        {
            if (!s.fresh_pending)
            {
                s.fresh_pending  = true;
                s.fresh_since_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - s.fresh_since_ms) >= kHoldMs)
            {
                s.asserted      = false;
                s.fresh_pending = false;
            }
        }
    }
    return s.asserted;
}

}  // namespace sensor_staleness
