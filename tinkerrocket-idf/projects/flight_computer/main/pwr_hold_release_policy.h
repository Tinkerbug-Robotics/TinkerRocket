#pragma once

// #1137 item 9 — may the FC hand its P4_EN_HOLD back at LANDED?
//
// The hold keeps the FC's own rail up independently of the OC's PWR_PIN.
// Releasing it drives GPIO5 low (high-Z through D9), after which R84/C105
// decay the rail in ~0.8 s unless the OC is actually driving it.  So the
// release is only safe when the OC is alive, and the comment at the release
// site has always said as much: "the hold is what keeps this downed rocket's
// GNSS downlink alive".
//
// It was gated on `out_ready`, which cannot answer that question.  out_ready
// is a set-once latch, and the I2C status poll that sets it is gated off for
// the entire INFLIGHT phase — so at landing it reports whether the OC was
// alive BEFORE launch.  For any rocket whose OC answered on the pad it is
// unconditionally true at LANDED, including one whose OC died at burnout, so
// the release was effectively unconditional in exactly the case it was written
// to protect against.
//
// The fix is to ask for evidence from after launch.  There is none at the
// LANDED transition itself — the poll only re-opens because the state is no
// longer INFLIGHT — so a healthy rocket takes the keep branch briefly and the
// #848 orphan-keep reconciliation hands the rail back a poll or two later.
// That asymmetry is deliberate: keeping the hold too long costs a battery
// pull, releasing it too early costs the rocket's downlink where it landed.

#include <stdint.h>

namespace pwr_hold
{

/// True when the OC has answered a status poll since the vehicle left the pad.
///
/// All three arguments are millis()-domain; the comparison is a signed
/// difference so the ~49.7-day rollover is a non-event.  A flight that never
/// launched leaves `launch_ms` at 0, which makes any real answer qualify --
/// correct, since with no flight there is no window during which the poll was
/// suppressed.
inline bool ocAliveSinceLaunch(bool     out_ready,
                               uint32_t out_ready_last_ms,
                               uint32_t launch_ms)
{
    return out_ready && (int32_t)(out_ready_last_ms - launch_ms) > 0;
}

}  // namespace pwr_hold
