#pragma once

#include <cmath>
#include <stdint.h>

// Whether coast guidance may steer this tick, why not, and where it measures
// "the pad" from.
//
// GTV Blue and GTV White, 2026-11-01 (fw 41b2ab92), the first two flights to
// arm station-keeping, showed three things the old one-line predicate in the
// flight loop could not:
//
//   1. THE ORIGIN WAS NOT THE PAD.  Guidance was handed imu_pos, which is
//      relative to the ENU reference datum -- the average of the first two
//      minutes of GNSS fixes after power-up, then frozen.  Both boards were
//      powered at the prep table, so on the rail imu_pos read (6.7, -42.4) and
//      (8.1, -44.4) m.  Station-keeping regulates position to (0,0): at
//      kp = 0.8 /s^2 a 43 m error commands 34 m/s^2, saturating the 20 m/s^2
//      budget toward the prep table from the first tick, whatever the rocket
//      was doing.  The origin is now the launch point itself, taken from the
//      filter at launch (launchOrigin below).
//
//   2. THE HEADING WAS WRONG.  Blue's magnetometer read 86 uT against the
//      model's 52 (its hard iron did not fit the vehicle; the FC had it amber
//      the whole pad wait) and its filter heading was ~90 deg off in coast.
//      Every lateral command is rotated into the body by that heading, so a
//      90 deg error turns the position loop's restoring push sideways and
//      anything past 90 deg makes it push outward.  Guidance now needs a
//      healthy magnetometer at launch (GF_NO_HEADING).
//
//   3. NOTHING SAID WHY IT DID NOT STEER.  White flew with guidance enabled;
//      it had weathercocked to 35.7 deg by the first control tick (T+0.6 s)
//      and the boost tilt latch (15 deg) held guidance off for the flight.
//      The log showed only "enabled, not active".  The reasons are now bits
//      in GuidanceTelemData::guid_flags, streamed while enabled.
//
// Pure, so it is host-testable in the style of roll_control_mode_policy.h.
namespace GuidanceGatePolicy {

// GuidanceTelemData::guid_flags.  Bits 0-1 keep their meaning; 2-7 were
// always written 0, so a decoder that reads only 0-1 reads the new frames as
// before.  A frame with no bit set but GF_ACTIVE clear is the legacy PN law
// past its closest point of approach (the only remaining reason, PN-only).
constexpr uint8_t GF_ACTIVE     = 1u << 0;  // steering this tick
constexpr uint8_t GF_BURNOUT    = 1u << 1;  // burnout detected
constexpr uint8_t GF_TILT       = 1u << 2;  // tilt-limit latch tripped (for the flight)
constexpr uint8_t GF_SLOW       = 1u << 3;  // speed at or below the guidance floor
constexpr uint8_t GF_EKF        = 1u << 4;  // filter never initialised, or not healthy
constexpr uint8_t GF_NO_ORIGIN  = 1u << 5;  // no position frame: no datum, or no launch origin
constexpr uint8_t GF_NO_HEADING = 1u << 6;  // heading not trusted at launch (for the flight)
constexpr uint8_t GF_HELD       = 1u << 7;  // fins held neutral: roll delay, speed gate,
                                            // reboot settle or stale IMU

constexpr uint8_t GF_REASONS = GF_TILT | GF_SLOW | GF_EKF | GF_NO_ORIGIN |
                               GF_NO_HEADING | GF_HELD;

struct Inputs
{
    bool  ekf_initialized;
    bool  ekf_healthy;      // GpsInsEKF::isHealthy(): finite, not freshly diverged
    bool  have_ref_pos;     // GNSS-anchored datum exists (#557)
    bool  origin_valid;     // launch origin captured this flight
    bool  heading_trusted;  // decided once, at launch
    bool  tilt_latched;
    float speed_mps;
    float min_speed_mps;
};

// Every reason guidance may not steer, in GF_* bits (0 = clear).  GF_HELD is
// the caller's: the fins-held branch never reaches this.
inline uint8_t inhibitFlags(const Inputs& in)
{
    uint8_t f = 0;
    if (in.tilt_latched)                          f |= GF_TILT;
    if (!(in.speed_mps > in.min_speed_mps))       f |= GF_SLOW;   // NaN speed is slow
    if (!in.ekf_initialized || !in.ekf_healthy)   f |= GF_EKF;
    if (!in.have_ref_pos || !in.origin_valid)     f |= GF_NO_ORIGIN;
    if (!in.heading_trusted)                      f |= GF_NO_HEADING;
    return f;
}

// The launch point in the datum's local ENU (m), from the filter's position at
// launch -- the same small-angle conversion that makes imu_pos, so guidance's
// pad-relative position is exactly imu_pos minus this.
struct Origin
{
    float e_m;
    float n_m;
    bool  valid;
};

inline Origin launchOrigin(double lat_rad, double lon_rad,
                           double ref_lat_rad, double ref_lon_rad,
                           double ref_alt_m,
                           bool have_ref_pos, bool ekf_initialized)
{
    constexpr double R_EARTH = 6378137.0;
    if (!have_ref_pos || !ekf_initialized) return { 0.0f, 0.0f, false };
    const float e = (float)((lon_rad - ref_lon_rad) * (R_EARTH + ref_alt_m) * cos(ref_lat_rad));
    const float n = (float)((lat_rad - ref_lat_rad) * (R_EARTH + ref_alt_m));
    if (!std::isfinite(e) || !std::isfinite(n)) return { 0.0f, 0.0f, false };
    return { e, n, true };
}

// Heading trust at launch: the magnetometer's own health verdict (the pad
// card's amber).  SH_OK only -- DEGRADED is a stale or ill-fitting hard iron
// (#1304's model check) and N/A is no magnetometer at all; neither gives the
// filter a heading guidance may rotate commands by.
inline bool launchHeadingTrusted(uint8_t mag_health_state, uint8_t sh_ok)
{
    return mag_health_state == sh_ok;
}

}  // namespace GuidanceGatePolicy
