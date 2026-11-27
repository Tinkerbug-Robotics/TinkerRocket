#pragma once

#include <stdint.h>

#include "RecoveryArmGate.h"

// #1776: burnout on a flight restored from a snapshot that has none.
//
// THE CASE.  When the OC boots from a power loss in flight with no RAM cache
// and no NAND frame to serve, it serves the flight token's frame, which is
// copied once, on the PRELAUNCH -> INFLIGHT edge (#1717 is why the NAND frame
// is missing on the V10 board).  The FC restores from it with launch set and
// burnout and apogee unset.  The apogee vote runs only once burnout_detected
// is set, and while the IMU is healthy the barometer backstop waits for it too
// (#1706).  The live detector (BurnoutDetector.h) latches on 50 ms of negative
// axial specific force: drag in the coast, or a section hanging nose-down
// under the drogue.  A section hanging from its forward end reads about +1 g
// on that axis for the whole descent, so burnout never latched, apogee was
// never called and the main never fired, whichever trigger mode it had.
//
// THE RULE.  On a restored flight whose burnout is unset, latch it once this
// boot's own sensors show the vehicle is past thrust, all at once:
//   - the recovery arm gate is Open: the flight is live, on this boot's own
//     evidence (RecoveryArmGate.h);
//   - a barometric descent of at least 5 m/s held 3 s, believed once the
//     altitude filter has had the gate's settle, OR free fall under 3 m/s^2
//     held 2 s, the gate's free-fall arm (Config::gate).  Free fall needs the
//     thrust to cancel the drag for 2 s; a climbing rocket reads a descent
//     only through a static-port error;
//   - the specific force's magnitude at or under 2 g on a fresh IMU, held
//     250 ms.  A drogue's swing or snatch above 2 g restarts the hold.
// Neither bar alone rules out thrust.  A fast motor reads well over 2 g, but
// a long-burn or low-thrust one can read 1-2 g late in its burn, and its
// tail-off passes through the bar on every flight.  The static port is the
// other half: under 2 g the rocket gains at most about 1 g of vertical
// acceleration, so a port error that grows with dynamic pressure (5 % of q on
// the project's port model) grows far slower than the climb, and only the
// step near Mach 1 can read as a descent (#1706: 30 m under the peak at
// 294 m/s up).  A step is over in well under a second, so the descent is held
// 3 s, three times the gate's 1 s arm.  Under the drogue that costs 2 s, about
// 40 m at 20 m/s.
// The evidence is accumulated here, from the restore, whichever arm opened
// the gate: a gate opened by its boost arm on a restore under thrust, or by
// its spin arm under a spinning drogue, still needs its own past-thrust
// evidence before burnout latches, and gets it under the drogue.
//
// WHAT IT ENABLES is only what a live burnout enables: the apogee vote, the
// backstop after its usual 1 s settle (BARO_BURNOUT_SETTLE_MS), and through
// apogee the triggers, once the gate has handed the apogee over
// (RecoveryArmGate::apogeeArmed).  It arms nothing itself.  An altitude-on-
// descent channel fires at its altitude.  A time-after-apogee channel counts
// from this boot's apogee call, which comes after the restore, so it fires
// that much late; after a low restore, perhaps not before landing.
//
// ON THE GROUND.  A stale restore on a bench can open the gate: its boost arm
// is marginal against a swing or a throw, and its spin arm against an
// airframe turned by hand (RecoveryArmGate.h's audit).  What keeps this latch
// shut there is its own evidence: a descent held 3 s is 15 m of continuous
// drop, and 2 s of free fall about 20 m.  (On a bench held slightly nose-down
// the live detector latches burnout anyway, within 50 ms.)
//
// WHAT IT LEAVES ALONE.  Once burnout is set, restored or latched by the live
// detector, the caller stops asking.  A normal flight never asks (no restore).
// With a stale IMU it never latches: the backstop then trusts the barometer
// without burnout (#556), and a 0 m/s^2 stand-in must not read as calm.
//
// Pure and header-only, beside RecoveryArmGate, so it is host-tested
// (tests_cpp/test_restored_burnout.cpp) with the real kinematic checks.

namespace RestoredBurnout {

// What showed the vehicle past thrust, for the log.
enum class Evidence : uint8_t {
    None     = 0,
    Descent  = 1,   // a barometric descent, held 3 s
    FreeFall = 2,   // free fall, the gate's free-fall arm
};

struct Config {
    // At or under 2 g of specific force, held.
    float    calm_max_ms2 = 2.0f * RecoveryArmGate::kG_MS2;
    uint32_t calm_hold_ms = 250;
    // The barometric descent: the gate's 5 m/s, held 3 s rather than its 1 s,
    // so a static-port step under a burning motor cannot read as one (above).
    uint32_t descent_hold_ms = 3000;
    // The descent rate, the free-fall arm and the barometer's settle are the
    // recovery arm gate's own.
    RecoveryArmGate::Config gate{};
};

struct State {
    // RecoveryArmGate::held accumulators: the start tick plus one, 0 when not
    // accumulating.
    uint32_t descent_since  = 0;
    uint32_t freefall_since = 0;
    uint32_t calm_since     = 0;
};

inline void reset(State& st) { st = State{}; }

// One tick, after RecoveryArmGate::step, on the gate's own inputs.
//
//   applies : a restored flight whose burnout is unset (the caller's
//             recovery_gate_active && !burnout_detected).  When false the
//             accumulators are cleared and nothing latches.
//   gate    : the recovery arm gate's state after this tick's step.
//
// Returns the evidence on the tick burnout should latch, None otherwise.  The
// caller latches burnout_detected and stamps its time; once it has, `applies`
// is false and this is inert.
inline Evidence step(State& st, bool applies, const RecoveryArmGate::State& gate,
                     const RecoveryArmGate::Inputs& in, const Config& cfg = Config{})
{
    if (!applies)
    {
        reset(st);
        return Evidence::None;
    }

    // The barometer is believed once the altitude filter, restarted cold by
    // this boot, has had the gate's settle (RecoveryArmGate::baro_settle_ms).
    const bool baro_ready = gate.seeded && in.baro_healthy &&
                            (uint32_t)(in.now_ms - gate.seed_ms) >= cfg.gate.baro_settle_ms;
    const bool descent = RecoveryArmGate::held(
        baro_ready && (-in.baro_rate_mps) >= cfg.gate.descent_mps,
        in.now_ms, cfg.descent_hold_ms, st.descent_since);
    const bool freefall = RecoveryArmGate::held(
        in.imu_fresh && in.accel_norm_ms2 <= cfg.gate.freefall_ms2,
        in.now_ms, cfg.gate.freefall_hold_ms, st.freefall_since);
    const bool calm = RecoveryArmGate::held(
        in.imu_fresh && in.accel_norm_ms2 <= cfg.calm_max_ms2,
        in.now_ms, cfg.calm_hold_ms, st.calm_since);

    if (!RecoveryArmGate::armingPermitted(gate) || !calm)
    {
        return Evidence::None;
    }
    if (descent)  return Evidence::Descent;
    if (freefall) return Evidence::FreeFall;
    return Evidence::None;
}

inline const char* evidenceName(Evidence e)
{
    switch (e)
    {
        case Evidence::Descent:  return "a barometric descent";
        case Evidence::FreeFall: return "free fall";
        default:                 return "none";
    }
}

}  // namespace RestoredBurnout
