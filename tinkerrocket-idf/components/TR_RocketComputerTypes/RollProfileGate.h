#ifndef ROLL_PROFILE_GATE_H
#define ROLL_PROFILE_GATE_H

// #1115: the roll-profile (cmd 26 / RollProfileData) acceptance policy and the
// degree wrap it protects, as PURE functions shared by both computers and
// host-tested (GuidancePointGate.h precedent).
//
// Two independent defects lived here, and this header closes both:
//
// 1. THE WRAP WAS NOT TOTAL.  The old wrap was
//        while (a >  180.0f) a -= 360.0f;
//        while (a < -180.0f) a += 360.0f;
//    which makes zero progress once |a| >= 2^33 (ulp(a) > 720, so a - 360
//    rounds straight back to a) and never progresses at all for +/-inf.  It
//    ran on the highest-priority flight task, inside the ~1 kHz loop, so a
//    single out-of-range waypoint angle wedged the task until the 5 s task
//    WDT panicked the FC -- and snapshot recovery then resumed the flight
//    clock at the same point in the profile and hung again, once per reboot,
//    for the rest of the flight.  Even a finite 1e8 terminated only after
//    ~278k iterations inside a 1 ms budget.  wrap180f() below is O(1),
//    total for every float, and exact.
//
// 2. THE INPUT WAS NEVER RANGE-CHECKED.  RollProfileData arrived from BLE as
//    76 raw bytes, was memcpy'd in with only num_waypoints clamped, and was
//    persisted to NVS verbatim -- so a typo in the app's free-text angle
//    field ("9999999999", a pasted "1e30" or "inf"; neither app clamps) was
//    stored, logged back to the operator as if accepted, and flown.
//    rollProfileSane() is the missing check.
//
// Both fixes are wanted: (1) alone removes the hang for every present and
// future caller of the wrap, including a target derived from a NaN attitude
// rather than from the profile; (2) alone stops the garbage at the door and
// keeps the FLOWN profile equal to the uploaded one.
//
// Policy is REJECT, never clamp -- the same rule as the guidance aim point
// (GuidancePointGate.h): a clamped profile flies something the operator never
// asked for and never sees, whereas a rejected one leaves the previous profile
// in place and says so in the log.  The config report keeps carrying what the
// rocket actually holds, so both apps replace the refused waypoint with the
// real one -- on their next ATTACH, since that is when they adopt.

#include <cmath>
#include <stdint.h>

#include "RocketComputerTypes.h"

// Largest |angle_deg| accepted in a waypoint.
//
// This is a SANITY bound, not a control bound.  A waypoint angle is a
// POSITION, and the profile interpolates along the shortest wrapped arc, so
// every value is equivalent to one in [-180, 180] and nothing above 360 can
// express anything new (cumulative angles do NOT spin the vehicle twice:
// 0 -> 360 -> 720 interpolates as 0 -> 0 -> 0).  720 rather than 360 because
// the bound exists to catch a typo, a paste, or a garbage float -- not to
// second-guess an operator who wrote 540 meaning 180, which flies identically
// once the wrap is total.  Any ten-digit keypad entry, any unit confusion and
// every non-finite value is still refused.
static constexpr float ROLL_WP_MAX_ABS_ANGLE_DEG = 720.0f;

// Largest waypoint time_s accepted, seconds after launch.
//
// Also a sanity bound.  The profile is consumed against
// t_flight = (now_ms - launch_time_millis)/1000, and roll control only runs
// while the fins have authority -- boost and early coast, single-digit
// seconds.  600 s is ~2 orders of magnitude past any flight this airframe
// makes, so it rejects a millisecond value sent as seconds, or a Unix
// timestamp, while never rejecting a real profile.  Negative times are
// refused outright: they name a moment before launch, which the query cannot
// reach.
static constexpr float ROLL_WP_MAX_TIME_S = 600.0f;

// Wrap a value in degrees to [-180, +180].  TOTAL: terminates in O(1) for
// every float, including +/-inf, NaN and 1e30.
//
// Non-finite in -> 0.0f out.  0 deg is the safe answer in both callers: as a
// profile target it commands "hold the reference roll", and as an angle error
// it commands zero rate, which is what the controller does anyway whenever
// the angle loop is not engaged.
//
// std::fmod is used rather than a loop or a floorf() closed form because it is
// EXACT (the IEEE remainder is always representable), so the identity path is
// bit-preserving for any input already in range -- including +/-180, which
// the old loops also left alone -- and a huge input cannot come back out of
// range through catastrophic cancellation the way `a - 360*floorf((a+180)/360)`
// can once ulp(a) exceeds 360.
static inline float wrap180f(float a)
{
    if (!std::isfinite(a))
    {
        return 0.0f;
    }
    a = std::fmod(a, 360.0f);      // exact; result in (-360, +360)
    if (a >  180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

// Reason a profile was refused.  Reported only in the FC/OC logs today; the
// enum exists so the log line names the failing rule instead of a bare
// "rejected", and so a future config-report field has something to carry.
enum RollProfileRc : uint8_t
{
    ROLL_PROF_OK          = 0,
    ROLL_PROF_REJ_COUNT   = 1,  // num_waypoints > MAX_ROLL_WAYPOINTS
    ROLL_PROF_REJ_TIME    = 2,  // a time_s is non-finite or outside [0, MAX]
    ROLL_PROF_REJ_ORDER   = 3,  // times are not non-decreasing
    ROLL_PROF_REJ_ANGLE   = 4,  // an angle_deg is non-finite or |angle| > MAX
};

// Validate a profile as received.  Only the first num_waypoints entries are
// examined: the wire frame is fixed-size and the unused tail is padding the
// query never reads, so a stale float there must not sink a good profile.
//
// num_waypoints == 0 is VALID and means rate-only (null roll) -- that is the
// documented empty profile and the fail-safe the rejecting callers fall back
// to, so it can never itself be a rejection.
//
// Times must be non-decreasing because roll_profile_query() walks the
// waypoints in order and takes the first segment whose end time is ahead of
// t_flight.  Out-of-order times do not hang, but they silently skip
// waypoints and produce a target the operator did not draw.  Equal times are
// allowed: they are the way to step the target discontinuously.
static inline RollProfileRc rollProfileRc(const RollProfileData &p)
{
    if (p.num_waypoints > MAX_ROLL_WAYPOINTS)
    {
        return ROLL_PROF_REJ_COUNT;
    }
    for (uint8_t i = 0; i < p.num_waypoints; ++i)
    {
        const float t = p.waypoints[i].time_s;
        const float a = p.waypoints[i].angle_deg;
        if (!std::isfinite(t) || t < 0.0f || t > ROLL_WP_MAX_TIME_S)
        {
            return ROLL_PROF_REJ_TIME;
        }
        if (i > 0 && t < p.waypoints[i - 1].time_s)
        {
            return ROLL_PROF_REJ_ORDER;
        }
        if (!std::isfinite(a) || std::fabs(a) > ROLL_WP_MAX_ABS_ANGLE_DEG)
        {
            return ROLL_PROF_REJ_ANGLE;
        }
    }
    return ROLL_PROF_OK;
}

static inline bool rollProfileSane(const RollProfileData &p)
{
    return rollProfileRc(p) == ROLL_PROF_OK;
}

// Short tag for the log line naming which rule refused the profile.
static inline const char *rollProfileRcName(RollProfileRc rc)
{
    switch (rc)
    {
        case ROLL_PROF_OK:        return "ok";
        case ROLL_PROF_REJ_COUNT: return "waypoint count";
        case ROLL_PROF_REJ_TIME:  return "waypoint time";
        case ROLL_PROF_REJ_ORDER: return "time order";
        case ROLL_PROF_REJ_ANGLE: return "waypoint angle";
    }
    return "unknown";
}

#endif // ROLL_PROFILE_GATE_H
