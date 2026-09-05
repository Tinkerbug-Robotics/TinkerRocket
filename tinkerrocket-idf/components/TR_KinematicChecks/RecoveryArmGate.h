#pragma once

#include <stdint.h>

// #1176: may a flight restored by reboot recovery arm a deployment channel?
//
// THE PROBLEM THIS SOLVES.  Reboot recovery restores rocket_state = INFLIGHT
// and the apogee latch from a stored snapshot, and servicePyroChannels() then
// evaluates the trigger conditions on the next tick.  Every input to that
// decision came from the PREVIOUS boot; nothing in it is evidence that the
// vehicle is still flying NOW.  If the stored record is stale — a flight that
// ended without reaching LANDED, a battery reseat, a recovered airframe on the
// bench — the restore happens on the ground with people around it, and a
// time-after-apogee channel that is already due fires immediately.
//
// So the restore decision is split in two, with opposite failure directions:
//
//   LIVENESS  — come back alive, restore state, log and transmit.  Fails
//               toward YES.  A false positive costs a skipped self-test.
//               Decided elsewhere (the durable flight token).
//   ARMING    — may a channel leave Idle?  Fails toward NO.  A false positive
//               is the one that hurts people.  Decided HERE, and only on
//               measurements taken by THIS boot's own sensors.
//
// WHAT THIS GATE MAY AND MAY NOT USE.  Owner's ruling, 2026-09-05, and it is
// a hard constraint rather than a preference: after a reboot there is nothing
// to integrate and GNSS is ~30 s from a fix, so absolute altitude is the one
// quantity the vehicle CANNOT establish.  A stale token supplies a foreign
// ground-pressure reference, and barometric AGL computed against it is wrong
// by hundreds of metres.  A RATE, by contrast, merely scales with that
// reference: a few percent of reference error costs a few percent of rate
// error.  Every arm below is therefore a rate or an acceleration.  None is an
// altitude, and none may become one.
//
// This also independently kills the "travel since boot" arm an earlier draft
// carried (40 m of excursion at under 5 m/s): a ridge landing, a hillside
// walk or a few flights of stairs satisfies it.
//
// SECOND OWNER RULING, same date: "There is no need to arm if the rocket is
// under a main chute. If we are slow or low don't come back and arm any
// charges."  So slow descent is a REFUSAL, not an arm.  Under a main at
// ~5 m/s nothing arms; under a drogue at ~15 m/s the gate opens and the main
// charge still fires, which is the case that actually matters.
//
// THIRD OWNER RULING, same date: there is NO sensorless backstop.  A recovery
// boot that comes back with both the barometer and the IMU dead never arms.
// The cost is accepted and stated: a genuine flight that loses both sensors
// across the reboot makes no deployment.  In exchange the gate has no path to
// Open that is not backed by a live measurement, which is what makes the
// ground case structurally safe rather than merely unlikely.
//
// GROUND-HANDLING AUDIT.  Each arm is sized so that handling a recovered
// airframe cannot satisfy it.  Handling is TRANSIENT; flight is SUSTAINED,
// so every arm carries a continuous hold requirement and the marginal ones
// carry a longer hold.  Per arm:
//
//   A1 descent   5 m/s held 1 s  = 5 m of continuous drop.  A dropped rocket
//                reaches the rate but hits the ground first; sustaining it
//                needs a fall of more than 5 m.  SAFE.
//   A2 boost     30 m/s^2 held 1 s.  MARGINAL, and knowingly so: #258's
//                launch-detect analysis records that a sustained SWING holds
//                centripetal >3 g steadily, which is the one non-oscillatory
//                false positive at this bar.  Bumps, knocks and carrying all
//                cross the reset floor and can never accumulate.
//   A4 GNSS      5 m/s vertical.  Not producible on the ground at all, but
//                unavailable for the first ~30 s, so it is a late corroborator
//                rather than a primary arm.
//   A5 free-fall |a| under 3 m/s^2 held 2 s.  The dwell is the whole defence:
//                free fall for 2 s needs a drop of about 20 m.  An earlier
//                draft used 500 ms, which needs only 1.2 m — i.e. dropping the
//                rocket on the pad would have armed it.  Do not shorten this
//                without re-doing that arithmetic.
//   A6 spin      250 dps held 2 s.  THE WEAKEST ARM, and the one to rule on
//                first at the bench.  It exists because it is the ONLY arm
//                available in the baro-dead-under-drogue case: the vehicle is
//                at 1 g (so A5 is out) and descending (so A1 needs the
//                barometer it does not have), and the main charge still has
//                to fire.  250 dps is ~42 rpm; a person can rotate an airframe
//                that fast by hand, so the 2 s hold is doing real work here.
//
// FAILURE MODE IS REFUSAL, which on the power-on path is bit-identical to
// today's behaviour (no recovery at all).  The gate can therefore never be a
// regression on that path; it can only add capability.
//
// Pure and header-only so the decision table is host-testable, in
// TR_KinematicChecks alongside MainDeployGate so the mini shares it.

namespace RecoveryArmGate {

enum class Verdict : uint8_t {
    Locked  = 0,   // no live evidence yet — no channel may leave Idle
    Open    = 1,   // live evidence of flight; arming permitted (latched)
    Refuted = 2,   // positive evidence the vehicle is NOT flying
};

// Which arm opened the gate.  Recorded for the log and post-flight forensics:
// "it opened" is not a useful answer on its own when the thresholds are still
// being tuned against real descents.
enum class Arm : uint8_t {
    None = 0, Descent = 1, Boost = 2, Gnss = 3, FreeFall = 4, Spin = 5,
};

struct Config {
    // --- Arming evidence.  Rates and accelerations only; see the header note.
    float    descent_mps      = 5.0f;    // A1, magnitude, down-positive sense
    uint32_t descent_hold_ms  = 1000;
    float    boost_ms2        = 30.0f;   // A2, |specific force|
    uint32_t boost_hold_ms    = 1000;
    float    gnss_mps         = 5.0f;    // A4, |vertical speed|
    uint32_t gnss_hold_ms     = 1000;
    float    freefall_ms2     = 3.0f;    // A5, |specific force| BELOW this
    uint32_t freefall_hold_ms = 2000;    // see the audit — do not shorten
    float    spin_dps         = 250.0f;  // A6, |angular rate|
    uint32_t spin_hold_ms     = 2000;

    // The barometric rate comes from an altitude filter that restarts cold on
    // a recovery boot and free-runs until it has real samples, so give it a
    // settle before any baro-derived arm is believed.  Mirrors
    // MainDeployGate's reacquire dwell, and for the same reason.
    uint32_t baro_settle_ms   = 600;

    // --- Ground refutation.  POSITIVE stillness evidence only, and only from
    // the shipped quiescence detector (TR_KinematicChecks): a blocked static
    // port emits positive-LOOKING evidence — fresh, in band, zero rate — so a
    // bespoke at-rest test built on the barometer would refute a live flight
    // whose port iced over.  quiescent_flag is baro-independent, calibrated
    // against real flight data, and is already what MainDeployGate trusts.
    uint32_t refute_hold_ms   = 30000;

    // A restored flight older than the FC's own flight timeout is over by
    // definition, whatever the sensors say.
    uint32_t max_flight_ms    = 600000;  // = FC MAX_FLIGHT_TIME_MS

    // Open must PERSIST this long before the apogee latch is handed to the
    // trigger conditions.  A second checkpoint on the same evidence, so a
    // single noisy tick that trips an arm cannot also arm the charge.
    uint32_t apogee_arm_ms    = 1000;
};

struct State {
    Verdict  verdict      = Verdict::Locked;
    Arm      opened_by    = Arm::None;
    bool     seeded       = false;
    uint32_t seed_ms      = 0;
    uint32_t open_since   = 0;   // tick at which verdict became Open
    // Per-arm continuous-hold accumulators, each holding the tick at which the
    // condition most recently became true PLUS ONE; 0 means "not accumulating".
    // The +1 encoding is load-bearing — see held() for what it prevents.
    uint32_t descent_since  = 0;
    uint32_t boost_since    = 0;
    uint32_t gnss_since     = 0;
    uint32_t freefall_since = 0;
    uint32_t spin_since     = 0;
    uint32_t quiet_since    = 0;
};

struct Inputs {
    uint32_t now_ms            = 0;
    uint32_t flight_elapsed_ms = 0;   // now - launch_time_millis, restored

    // Barometric.  Same predicate MainDeployGate consumes (#257: fresh and
    // in-range).  Rate is UP-POSITIVE, matching kinematics.d_alt_est_, so a
    // descent is negative.
    bool     baro_healthy   = false;
    float    baro_rate_mps  = 0.0f;

    // Inertial.  accel_norm is a specific-force magnitude in m/s^2 — pad
    // gravity reads about 9.71, NOT 1.0.  gyro_norm is a magnitude in deg/s.
    // imu_fresh must be the FRESHNESS gate, not merely "a sample exists":
    // ism6_latest_si retains its last value when the drain yields nothing, so
    // a frozen IMU would otherwise re-present one stale sample forever.
    bool     imu_fresh      = false;
    float    accel_norm_ms2 = 0.0f;
    float    gyro_norm_dps  = 0.0f;

    // GNSS.  Up-positive vertical speed; unavailable for the first ~30 s.
    bool     gnss_ok        = false;
    float    gnss_vel_u_mps = 0.0f;

    // The shipped baro-independent stillness detector.
    bool     quiescent_flag = false;
};

inline void reset(State& st, uint32_t now_ms)
{
    st = State{};
    st.seed_ms = now_ms;
    st.seeded  = true;
}

// One continuous-hold accumulator.  Returns true once `cond` has been
// unbroken for `hold_ms`.
//
// `since_p1` stores the start tick PLUS ONE, so that 0 can mean "not
// accumulating" without colliding with a genuine start at now_ms == 0.  An
// earlier version stored the raw tick and mapped 0 to 1, which made every arm
// mature on the first tick of a boot: the elapsed time came out as 0 - 1, i.e.
// 4294967295 ms, which clears any hold.  The host tests caught it, and they
// are the reason this comment exists — do not "simplify" it back.
//
// Wrap-safe: the subtraction is modular, so the only cost at the 49-day
// rollover is one restarted accumulator when now_ms is exactly UINT32_MAX.
inline bool held(bool cond, uint32_t now_ms, uint32_t hold_ms, uint32_t& since_p1)
{
    if (!cond) { since_p1 = 0; return false; }
    if (since_p1 == 0) { since_p1 = now_ms + 1u; }
    return (uint32_t)(now_ms - (since_p1 - 1u)) >= hold_ms;
}

// Advance the gate.  Once per tick, before the trigger conditions are
// evaluated.  Open and Refuted are both terminal for the flight.
inline void step(State& st, const Inputs& in, const Config& cfg = Config{})
{
    if (!st.seeded) reset(st, in.now_ms);
    if (st.verdict != Verdict::Locked) return;   // both terminal

    // A restored flight past the FC's own timeout is over whatever the
    // sensors say.  Checked before the arms so a stale token that also
    // happens to be moving cannot open.
    if (in.flight_elapsed_ms >= cfg.max_flight_ms)
    {
        st.verdict = Verdict::Refuted;
        return;
    }

    // Refutation: sustained, positive, baro-independent stillness.
    if (held(in.quiescent_flag, in.now_ms, cfg.refute_hold_ms, st.quiet_since))
    {
        st.verdict = Verdict::Refuted;
        return;
    }

    // Baro-derived evidence is only believed once the altitude filter has had
    // time to re-converge on this boot's own samples.
    const bool baro_ready =
        in.baro_healthy &&
        (uint32_t)(in.now_ms - st.seed_ms) >= cfg.baro_settle_ms;

    // A1 — sustained descent.  Down-positive comparison on an up-positive rate.
    if (held(baro_ready && (-in.baro_rate_mps) >= cfg.descent_mps,
             in.now_ms, cfg.descent_hold_ms, st.descent_since))
    {
        st.verdict = Verdict::Open; st.opened_by = Arm::Descent;
        st.open_since = in.now_ms; return;
    }
    // A2 — sustained specific force well above 1 g.
    if (held(in.imu_fresh && in.accel_norm_ms2 >= cfg.boost_ms2,
             in.now_ms, cfg.boost_hold_ms, st.boost_since))
    {
        st.verdict = Verdict::Open; st.opened_by = Arm::Boost;
        st.open_since = in.now_ms; return;
    }
    // A4 — GNSS vertical speed, either sign: a reboot can land anywhere in the
    // profile, and climbing is as much proof of flight as descending.
    {
        const float v = in.gnss_vel_u_mps < 0.0f ? -in.gnss_vel_u_mps
                                                 : in.gnss_vel_u_mps;
        if (held(in.gnss_ok && v >= cfg.gnss_mps,
                 in.now_ms, cfg.gnss_hold_ms, st.gnss_since))
        {
            st.verdict = Verdict::Open; st.opened_by = Arm::Gnss;
            st.open_since = in.now_ms; return;
        }
    }
    // A5 — sustained free fall.  Unsupported flight before the chute is out.
    if (held(in.imu_fresh && in.accel_norm_ms2 <= cfg.freefall_ms2,
             in.now_ms, cfg.freefall_hold_ms, st.freefall_since))
    {
        st.verdict = Verdict::Open; st.opened_by = Arm::FreeFall;
        st.open_since = in.now_ms; return;
    }
    // A6 — sustained spin.  The only arm available with a dead barometer under
    // a drogue; see the audit in the header before touching its numbers.
    if (held(in.imu_fresh && in.gyro_norm_dps >= cfg.spin_dps,
             in.now_ms, cfg.spin_hold_ms, st.spin_since))
    {
        st.verdict = Verdict::Open; st.opened_by = Arm::Spin;
        st.open_since = in.now_ms; return;
    }
}

// May a deployment channel leave Idle at all?
inline bool armingPermitted(const State& st)
{
    return st.verdict == Verdict::Open;
}

// May the restored apogee latch be handed to the trigger conditions?  Open
// must have PERSISTED, so one noisy tick cannot both open the gate and arm a
// charge that is already past its time-after-apogee delay.
//
// This is deliberately separate from the restore itself: pyro_apogee_detected
// is restored normally so the snapshot keeps round-tripping (a second reboot
// must not read back a flight with its apogee erased), and this bool gates the
// two trigger conditions that consume it.
inline bool apogeeArmed(const State& st, uint32_t now_ms,
                        const Config& cfg = Config{})
{
    return st.verdict == Verdict::Open &&
           (uint32_t)(now_ms - st.open_since) >= cfg.apogee_arm_ms;
}

// Positive evidence the vehicle is not flying: the caller drives LANDED.
inline bool refuted(const State& st) { return st.verdict == Verdict::Refuted; }

inline const char* armName(Arm a)
{
    switch (a)
    {
        case Arm::Descent:  return "descent";
        case Arm::Boost:    return "boost";
        case Arm::Gnss:     return "gnss";
        case Arm::FreeFall: return "free-fall";
        case Arm::Spin:     return "spin";
        default:            return "none";
    }
}

}  // namespace RecoveryArmGate
