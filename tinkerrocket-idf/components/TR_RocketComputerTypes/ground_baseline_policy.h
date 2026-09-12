#pragma once

// #1150 — when the comms side may take a ground-pressure reference, and where
// it gets one when it joined a flight already in the air.
//
// The out computer (and the mini's comms half, a verbatim descendant) derives
// the pressure altitude it puts on LoRa and BLE from its own copy of the BMP
// stream against a ground reference it tracks itself: every baro sample that
// arrives while the flight side is not INFLIGHT becomes the new reference.
// That state is a plain static that reads INITIALIZATION from boot until the
// first NonSensorData frame lands, and the baro and NonSensor frames run at
// the same rate with the baro frame earlier in each flight-loop pass.  So an
// out computer that reset in flight (the #825/#1176 rail-restore case) and
// re-joined the FC's stream took its "pad" reference from whichever baro
// frame beat the first NonSensor frame -- at altitude, more often than not.
// From then on pressure_alt read ~0 and went negative on descent, max_alt
// restarted from 0, and nothing logged it: the operator saw a rocket sitting
// at 0 m while it was still 800 m up.
//
// Two rules, both pure so the host suite can pin them:
//
//   1. trackFromBaro -- a baro sample may become the reference only while the
//      flight side's state is KNOWN (a NonSensor frame has been seen this
//      boot) and not INFLIGHT.  The default-initialised enum is not evidence.
//
//   2. adoptFromSnapshot -- the flight side already carries the reference the
//      comms side lost.  FlightSnapshotData (v5) rides the same link at 10 Hz
//      for the whole of INFLIGHT with ground_pressure_pa, max_alt_m and
//      max_speed_mps in it; the flight side froze that reference at PRELAUNCH
//      (#1108) and restores it across its own reboots (#846).  A comms side
//      that is INFLIGHT with no reference of its own adopts the snapshot's,
//      ONCE.  A comms side that watched the pad keeps what it tracked: the
//      flight side's reference is frozen at PRELAUNCH entry, so a long pad
//      wait lets weather drift it, while the comms side tracks until launch
//      detection -- metres either way, and this keeps the nominal flight's
//      numbers exactly what they were.
//
// Rule 1 alone would close the hole but leave the altitude at 0 for the rest
// of the flight (no reference, no altitude), which is the same telemetry the
// operator sees today minus the sign.  Rule 2 is what makes the restarted
// board useful again within one snapshot period.

#include <stdint.h>

#include "RocketComputerTypes.h"

namespace ground_baseline
{

// The only altitudes and speeds a derived maximum may latch.  The same bands
// the OC applied inline to its own maxima before #1150 (a corrupt frame must
// not become the flight's apogee); shared here so the adoption path and the
// live path cannot drift apart.
inline constexpr float kMaxAltMinM  = -500.0f;
inline constexpr float kMaxAltMaxM  = 100000.0f;
inline constexpr float kMaxSpeedMps = 1500.0f;

/// Rule 1.  `fc_state_known` is "a NonSensor frame has been seen this boot";
/// `fc_inflight` is the state that frame carried.
inline bool trackFromBaro(bool fc_state_known, bool fc_inflight)
{
    return fc_state_known && !fc_inflight;
}

enum class Verdict : uint8_t
{
    Adopt,         // no reference for this flight: take the snapshot's
    HaveBaseline,  // the comms side saw the pad itself -- keep its own
    NotInflight,   // a LANDED clear, or anything else: nothing to adopt from
    Rejected,      // magic, version or pressure band failed: not trusted
};

struct Adoption
{
    float ground_pressure_pa = 0.0f;
    float max_alt_m          = 0.0f;   // 0 = contributes nothing to a max()
    float max_speed_mps      = 0.0f;   // 0 = contributes nothing to a max()
};

/// Rule 2.  Called for every accepted snapshot frame; `own_baseline_set` is
/// the comms side's "I have a reference for the current flight".  On Adopt,
/// `out` holds the reference to install and the maxima to fold in with
/// max(); an out-of-band maximum comes back as 0 so it folds in as nothing
/// rather than costing the reference.
inline Verdict adoptFromSnapshot(bool own_baseline_set,
                                 const FlightSnapshotData& snap,
                                 Adoption& out)
{
    if (snap.magic != FlightSnapshotData::MAGIC ||
        snap.version != FlightSnapshotData::VERSION)
    {
        return Verdict::Rejected;
    }
    if (snap.rocket_state != (uint8_t)INFLIGHT)
    {
        return Verdict::NotInflight;
    }
    if (own_baseline_set)
    {
        return Verdict::HaveBaseline;
    }
    // Negated form so NaN fails too.
    if (!(snap.ground_pressure_pa >= BMP_PRESSURE_MIN_PA &&
          snap.ground_pressure_pa <= BMP_PRESSURE_MAX_PA))
    {
        return Verdict::Rejected;
    }
    out.ground_pressure_pa = snap.ground_pressure_pa;
    out.max_alt_m =
        (snap.max_alt_m > kMaxAltMinM && snap.max_alt_m < kMaxAltMaxM)
            ? snap.max_alt_m : 0.0f;
    out.max_speed_mps =
        (snap.max_speed_mps >= 0.0f && snap.max_speed_mps <= kMaxSpeedMps)
            ? snap.max_speed_mps : 0.0f;
    return Verdict::Adopt;
}

}  // namespace ground_baseline
