#ifndef GUIDANCE_POINT_GATE_H
#define GUIDANCE_POINT_GATE_H

// #435: the cmd-28 (Drift-Cast guidance aim point) acceptance policy as a
// PURE function, extracted from the FC handler so the gate ORDER and its
// edge cases are host-testable (MramDirtyPolicy precedent).  The FC handler
// computes the inputs (state, lockout, law, ref availability, the converted
// pad-relative radius) and this function alone decides the GUID_RC_* result.
//
// Gate order is LOAD-BEARING and pinned by the host tests:
//   state > badval > law > noref > radius
// - state first: a point delivered in the wrong state must never leak a more
//   specific (and more actionable-sounding) rejection reason.
// - badval before law/noref: garbage coordinates are garbage regardless of
//   configuration.
// - law before noref: accepting a point that PN would silently ignore
//   recreates the #376 lie, and that answer doesn't depend on GNSS.
// - radius last: it is the only check that needs the conversion, which in
//   turn needs the reference (noref already screened that).
//
// r_m is the caller-converted pad-relative horizontal distance; it is only
// consulted once every earlier gate passed, so callers without a reference
// may pass any value (conventionally 0).  Radius policy is REJECT-never-
// clamp, <= max_r_m accepts (the 100.0 m boundary point is valid).
//
// guidance_enabled is deliberately NOT an input: cmd 32 (enable) may
// legitimately arrive after the point, and "ge" is separately echoed.

#include <cmath>
#include <stdint.h>

#include "RocketComputerTypes.h"

inline uint8_t guidancePointRc(bool ready_or_prelaunch, bool lockout,
                               uint8_t law, bool have_ref,
                               double lat_deg, double lon_deg, float alt_m,
                               float r_m, float max_r_m)
{
    if (!ready_or_prelaunch || lockout)
    {
        return GUID_RC_REJ_STATE;
    }
    if (!std::isfinite(lat_deg) || !std::isfinite(lon_deg) ||
        !std::isfinite(alt_m) ||
        lat_deg < -90.0 || lat_deg > 90.0 ||
        lon_deg < -180.0 || lon_deg > 180.0)
    {
        return GUID_RC_REJ_BADVAL;
    }
    if (law != GUIDE_LAW_STATION_KEEP)
    {
        return GUID_RC_REJ_LAW;
    }
    if (!have_ref)
    {
        return GUID_RC_REJ_NOREF;
    }
    if (!std::isfinite(r_m) || r_m > max_r_m)
    {
        return GUID_RC_REJ_RADIUS;
    }
    return GUID_RC_ACCEPTED;
}

// ---------------------------------------------------------------------------
// WGS84 geodetic -> pad-relative flat-earth ENU (#435).  This is the ONLY
// conversion the cmd-28 path uses (receipt + launch re-convert), extracted
// pure so the axis convention and the radians contract are host-pinned
// against known geodesy values — an E/N transpose or a degrees-as-radians
// slip is radius-invariant, so no runtime gate can catch it.  Must agree
// term-for-term with the EKF position path in FC main.cpp (loop_fc imu_pos
// block) or the flown miss distance silently differs from the commanded one.
// lat/lon in DEGREES; ref_lat/lon in RADIANS; ref_alt in metres.
inline void guidanceLlaToEnu(double lat_deg, double lon_deg,
                             double ref_lat_rad, double ref_lon_rad,
                             double ref_alt_m,
                             float& e_m, float& n_m)
{
    constexpr double R_EARTH  = 6378137.0;
    constexpr double DEG2RAD  = 0.017453292519943295;  // pi/180
    const double lat_rad = lat_deg * DEG2RAD;
    const double lon_rad = lon_deg * DEG2RAD;
    e_m = (float)((lon_rad - ref_lon_rad) * (R_EARTH + ref_alt_m) * cos(ref_lat_rad));
    n_m = (float)((lat_rad - ref_lat_rad) * (R_EARTH + ref_alt_m));
}

// Launch-time re-check of an accepted Drift-Cast point against the just-
// frozen reference (#435).  Same REJECT-never-clamp policy as the receipt
// gate: keep the re-converted point iff its radius is finite and inside
// max_r_m, otherwise fail SAFE to (0,0) overhead and drop the point
// (point_kept=false -> caller clears guid_point_valid so the settings
// snapshot + echo record what actually flies).  Extracted pure so the
// keep-vs-wipe decision — a second acceptance policy — gets the same host
// coverage as guidancePointRc().
struct GuidanceLaunchTarget
{
    float e_m;
    float n_m;
    bool  point_kept;   // false = fail-safed to (0,0) overhead
};

inline GuidanceLaunchTarget guidanceLaunchReconvert(double lat_deg, double lon_deg,
                                                    double ref_lat_rad,
                                                    double ref_lon_rad,
                                                    double ref_alt_m,
                                                    float max_r_m)
{
    float e = 0.0f, n = 0.0f;
    guidanceLlaToEnu(lat_deg, lon_deg, ref_lat_rad, ref_lon_rad, ref_alt_m, e, n);
    const float r = sqrtf(e * e + n * n);
    if (std::isfinite(r) && r <= max_r_m)
    {
        return { e, n, true };
    }
    return { 0.0f, 0.0f, false };
}

// ---------------------------------------------------------------------------
// Guidance-target echo transitions (#435), pure over OutStatusQueryData so the
// handler wiring rules are host-testable: the gate verdict must be honored
// (a reject NEVER applies the point), seq bumps on EVERY processed cmd 28,
// and a reject leaves the display fields (status/lat/lon/alt — the still-
// active previous target) untouched.

// Apply a processed cmd-28 result to the echo.  Returns true iff the point
// was ACCEPTED — the caller applies it to the flight target ONLY then.
inline bool guidancePointEchoApply(OutStatusQueryData& q, uint8_t rc,
                                   double lat_deg, double lon_deg, float alt_m)
{
    const bool accepted = (rc == GUID_RC_ACCEPTED);
    if (accepted)
    {
        q.tgt_status  = GUID_TGT_GEO_ACTIVE;
        q.tgt_lat_deg = (float)lat_deg;
        q.tgt_lon_deg = (float)lon_deg;
        q.tgt_alt_m   = (int16_t)std::lround((double)alt_m);
    }
    // On reject: status/lat/lon/alt stay UNCHANGED — a still-active previous
    // target keeps displaying; only seq/last_rc report the result (why the
    // two are separate fields).
    q.tgt_seq++;
    q.tgt_last_rc = rc;
    return accepted;
}

// A non-cmd-28 event superseded any geodetic point (cmd-65 apply, sim
// start/stop reset, launch fail-safe): show new_status with cleared
// coordinates.  Bumps seq ONLY when the echo content actually changed, so a
// no-op connect-time profile push (no cmd-28 point in play) doesn't churn
// the app's seq baseline.  Deliberately does NOT touch tgt_last_rc: rc is
// the result of the last processed cmd 28, and this event isn't one.
// Returns true when the echo changed.
inline bool guidanceTargetEchoSupersede(OutStatusQueryData& q, uint8_t new_status)
{
    if (q.tgt_status == new_status &&
        q.tgt_lat_deg == 0.0f && q.tgt_lon_deg == 0.0f && q.tgt_alt_m == 0)
    {
        return false;
    }
    q.tgt_status  = new_status;
    q.tgt_lat_deg = 0.0f;
    q.tgt_lon_deg = 0.0f;
    q.tgt_alt_m   = 0;
    q.tgt_seq++;
    return true;
}

// Which target actually flies / is currently active (#435): a valid cmd-28
// geodetic point wins, else the cmd-65 profile point when target_mode is
// POINT, else overhead.  Shared by the FlightSettings v6 tail (flown src),
// the boot echo init, and the cmd-65 supersede status.
inline uint8_t guidanceFlownTargetSrc(bool geo_point_valid, uint8_t target_mode)
{
    return geo_point_valid
               ? GUID_TGT_GEO_ACTIVE
               : ((target_mode == GUIDE_TARGET_POINT) ? GUID_TGT_EN_ACTIVE
                                                      : GUID_TGT_NONE);
}

#endif  // GUIDANCE_POINT_GATE_H
