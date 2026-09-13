#ifndef SERVO_CONFIG_GATE_H
#define SERVO_CONFIG_GATE_H

// The SERVO_CONFIG (cmd 12 / ServoConfigData) acceptance policy, as PURE
// functions shared by both computers and host-tested (RollProfileGate.h
// precedent, and the same class of defect).
//
// WHY THIS MOVED OUT OF TR_ServoControl.  The predicates were already there,
// and their doc comments already said what they were for:
//
//     "Pure predicate, exposed so a caller can decide whether to PERSIST a
//      timing without applying it first (the OC caches config it never runs)."
//
// That caller is out_computer's cacheServoConfig(), and it never called them —
// it could not, because the OC does not link the servo driver.  So #1141 item
// 3 and #1137 item 2 landed on the FLIGHT computer only: the FC refuses a
// degenerate fin span or a timing that is not a servo timing, declines to
// persist it, and keeps flying the previous value, while the OC cached the
// refused bytes, wrote them to its OWN NVS, and went on reporting them as the
// rocket's configuration in the cmd-20 readback.
//
// MEASURED on the V9 bench 2026-09-12: push hz=0; the FC logs
//     [SERVO CFG] timing REJECTED: hz=0 min=1100 max=1900. Not applied, not
//     saved — the previous timing stands.
// and after a full FC power cycle restores hz=56 from its NVS — correct.  The
// app's readback said "shz":0 throughout, and still said 0 after the power
// cycle, because the OC had persisted it.  The operator is shown a servo rate
// the flight computer rejected and is not using, and it survives a reboot.
//
// Flight behaviour was never at risk: the FC validates independently and is
// the only thing that drives a servo.  What was wrong is the report, which is
// also what both apps ADOPT on attach — so the wrong value propagates into the
// phone's profile and is pushed back out from there.
//
// Policy is REJECT, never clamp — the same rule as the roll profile
// (RollProfileGate.h) and the guidance aim point (GuidancePointGate.h).  A
// clamped config runs something the operator never asked for and never sees;
// a rejected one leaves the previous value in place and says so in the log.
//
// The bounds are deliberately GENEROUS.  They reject values that are not servo
// timings at all, not values that are merely unusual — an operator running an
// unusual-but-real servo must not be second-guessed here.

#include <cmath>
#include <stdint.h>

// --- servo timing bounds -------------------------------------------------
//
// The duty math downstream is
//     duty = pulse_us * servo_hz * max_duty / 1000000
// so hz == 0 gives duty 0 on every channel — indistinguishable from idle(),
// i.e. no pulse train and four relaxed fins, applied silently from a wire
// value.  A negative hz casts to ~4.29e9 and the 32-bit product wraps to an
// arbitrary duty.  Neither is a servo timing; both used to be accepted.
static constexpr int SERVO_MIN_HZ             = 40;
static constexpr int SERVO_MAX_HZ             = 400;
static constexpr int SERVO_MIN_PULSE_US       = 500;
static constexpr int SERVO_MAX_PULSE_US       = 2500;
// A pulse span narrower than this is not a travel range — it is a servo that
// cannot move far enough to be told apart from one that is stuck.
static constexpr int SERVO_MIN_PULSE_SPAN_US  = 200;

// --- fin calibration bounds ----------------------------------------------
//
// Below this span the deg->pulse scale is so steep that a 1 deg command
// saturates the servo, which is indistinguishable from the frozen-fin failure
// this rejects.  A degenerate pair (min == max) divides by zero.
static constexpr float FIN_MIN_SPAN_DEG       = 2.0f;

// Reason a servo timing was refused.  Reported in the FC/OC logs so the line
// names the failing rule instead of a bare "rejected".
enum ServoTimingRc : uint8_t
{
    SERVO_TIMING_OK          = 0,
    SERVO_TIMING_REJ_HZ      = 1,  // hz outside [SERVO_MIN_HZ, SERVO_MAX_HZ]
    SERVO_TIMING_REJ_PULSE   = 2,  // an endpoint outside the pulse-width range
    SERVO_TIMING_REJ_SPAN    = 3,  // max_us - min_us < SERVO_MIN_PULSE_SPAN_US
};

// Reason a fin calibration was refused.
enum FinCalRc : uint8_t
{
    FIN_CAL_OK               = 0,
    FIN_CAL_REJ_NONFINITE    = 1,  // a NaN or +/-inf endpoint
    FIN_CAL_REJ_SPAN         = 2,  // max - min < FIN_MIN_SPAN_DEG (incl. <= 0)
};

static inline ServoTimingRc servoTimingRc(int hz, int minUs, int maxUs)
{
    if (hz < SERVO_MIN_HZ || hz > SERVO_MAX_HZ)
    {
        return SERVO_TIMING_REJ_HZ;
    }
    if (minUs < SERVO_MIN_PULSE_US || maxUs > SERVO_MAX_PULSE_US)
    {
        return SERVO_TIMING_REJ_PULSE;
    }
    if (maxUs - minUs < SERVO_MIN_PULSE_SPAN_US)
    {
        return SERVO_TIMING_REJ_SPAN;
    }
    return SERVO_TIMING_OK;
}

static inline bool servoTimingSane(int hz, int minUs, int maxUs)
{
    return servoTimingRc(hz, minUs, maxUs) == SERVO_TIMING_OK;
}

// Non-finite is tested FIRST and on its own: (max - min) with an inf or NaN
// endpoint is NaN, and every comparison against NaN is false, so a lone
// `(max - min) >= FIN_MIN_SPAN_DEG` already refuses it — but it refuses it
// without being able to say why, and the distinction is the difference
// between "your app sent garbage" and "your travel range is too narrow".
static inline FinCalRc finCalRc(float finMinDeg, float finMaxDeg)
{
    if (!std::isfinite(finMinDeg) || !std::isfinite(finMaxDeg))
    {
        return FIN_CAL_REJ_NONFINITE;
    }
    if ((finMaxDeg - finMinDeg) < FIN_MIN_SPAN_DEG)
    {
        return FIN_CAL_REJ_SPAN;
    }
    return FIN_CAL_OK;
}

static inline bool finCalSane(float finMinDeg, float finMaxDeg)
{
    return finCalRc(finMinDeg, finMaxDeg) == FIN_CAL_OK;
}

static inline const char *servoTimingRcName(ServoTimingRc rc)
{
    switch (rc)
    {
        case SERVO_TIMING_OK:        return "ok";
        case SERVO_TIMING_REJ_HZ:    return "hz out of range";
        case SERVO_TIMING_REJ_PULSE: return "pulse endpoint out of range";
        case SERVO_TIMING_REJ_SPAN:  return "pulse span too narrow";
    }
    return "unknown";
}

static inline const char *finCalRcName(FinCalRc rc)
{
    switch (rc)
    {
        case FIN_CAL_OK:            return "ok";
        case FIN_CAL_REJ_NONFINITE: return "non-finite endpoint";
        case FIN_CAL_REJ_SPAN:      return "span too narrow";
    }
    return "unknown";
}

#endif // SERVO_CONFIG_GATE_H
