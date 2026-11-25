#pragma once
// #1755: what the flight computer does with the boost thrust-axis check
// (TR_Orientation's ThrustAxisCheck) on an AUTO-mounted board.
//
// On AUTO the board-to-rocket frame is the last still pad window's.  A board
// snapped on a prep table, or mounted tail-forward and never confirmed on the
// rail, flies a lateral or reversed rocket +X, and burnout, the IMU's pitch
// apogee voter and roll control all read rocket +X.  The check measures the
// specific force's direction over the first 150 ms of the flight.  This policy
// decides whether that measurement disproves the frame, and so whether burnout
// reads the measured axis and the pitch voter and roll control stand down.
//
//   - The measurement is THRUST only if it points up: the IMU's own levelled
//     attitude (InertialVertical, frame-independent) gives its up component.
//     A flight entered at or after motor burnout (a late accel-only launch
//     detect, a short hot motor) measures coast drag, which points down; read
//     as the thrust axis, burnout would never latch.  With no levelled
//     attitude there is no way to tell, so no verdict.
//   - A thrust axis more than kMismatchDeg off rocket +X is a mismatch (the
//     telemetry flag NSF2_ORIENT_THRUST_MISMATCH), on any mounting.
//   - It DISPROVES the frame only on AUTO.  A manual code is the owner's
//     statement of the mounting (it also fixes the roll clocking gravity
//     cannot see) and keeps everything on rocket +X, as before: bench sims on
//     a manual code with the board lying flat put their thrust on rocket +/-Z.
//   - While the check is still measuring, AUTO burnout takes no input, at
//     most kWaitMs after launch (the 200 ms burnout lockout normally covers
//     the 150 ms window); a check still open then gives way to rocket +X.

//
// After a reboot in flight (follow-up to #1755) the verdict rides the OC's
// snapshot in FlightSnapshotData.b2r_mode's high bits, with the measured axis
// as its nearest rocket-frame axis (packSnapshotMode / restoredVerdict): a
// snapped frame's real nose IS one of those six axes, and on an AUTO_EXACT
// frame the nearest one still puts thrust and drag on the right side of
// zero.  And the roll rate every reader takes (rollRateDps) is the rate about
// the measured axis on a disproved frame, not gyro_x, which there is a pitch
// or yaw rate: the landing checks and the recovery arm gate read it too.

#include <cmath>
#include <cstdint>

#include "RocketComputerTypes.h"   // SNAP_B2R_*
#include "TR_Orientation.h"        // ORIENT_MODE_MANUAL

namespace ThrustAxisPolicy {

inline constexpr float    kMismatchDeg = 25.0f;
inline constexpr float    kMinUp       = 0.5f;    // thrust within 60 deg of up
inline constexpr uint32_t kWaitMs      = 1000;

struct Verdict
{
    bool thrust    = false;   // the measurement is the thrust axis
    bool mismatch  = false;   // ... and it is over kMismatchDeg off rocket +X
    bool disproved = false;   // ... on AUTO: the frame flown is wrong
};

// check_valid / angle_deg: the check's verdict.  inertial_healthy / axis_up:
// whether InertialVertical is flying, and the measured axis's up component.
inline Verdict judge(bool check_valid, float angle_deg, bool inertial_healthy,
                     float axis_up, bool manual_mounting)
{
    Verdict v;
    v.thrust    = check_valid && inertial_healthy && std::isfinite(axis_up) && axis_up >= kMinUp;
    v.mismatch  = v.thrust && std::isfinite(angle_deg) && angle_deg > kMismatchDeg;
    v.disproved = v.mismatch && !manual_mounting;
    return v;
}

// Hold AUTO burnout off while the axis is still being measured.
inline bool holdBurnout(bool manual_mounting, bool check_running, uint32_t t_since_launch_ms)
{
    return !manual_mounting && check_running && t_since_launch_ms < kWaitMs;
}

// The roll rate (dps, + about the nose): about the measured thrust axis once
// it has disproved the frame, rocket +X's gyro otherwise.
inline float rollRateDps(const float w_dps[3], bool disproved, const float axis[3])
{
    return disproved ? w_dps[0] * axis[0] + w_dps[1] * axis[1] + w_dps[2] * axis[2] : w_dps[0];
}

// ---- The verdict in the snapshot --------------------------------------------

// The nearest rocket-frame axis to a vector: 1-6 = +X, -X, +Y, -Y, +Z, -Z;
// 0 for a zero or non-finite one.
inline uint8_t axisCode(const float u[3])
{
    if (!(std::isfinite(u[0]) && std::isfinite(u[1]) && std::isfinite(u[2]))) return 0;
    int best = -1;
    float mag = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        if (std::fabs(u[i]) > mag)
        {
            mag = std::fabs(u[i]);
            best = i;
        }
    }
    if (best < 0) return 0;
    return (uint8_t)(1 + 2 * best + (u[best] < 0.0f ? 1 : 0));
}

// The unit vector of an axisCode; false (u untouched) for 0 or past 6.
inline bool axisFromCode(uint8_t code, float u[3])
{
    if (code < 1 || code > 6) return false;
    const int i = (code - 1) / 2;
    u[0] = u[1] = u[2] = 0.0f;
    u[i] = ((code - 1) % 2) ? -1.0f : 1.0f;
    return true;
}

// FlightSnapshotData.b2r_mode: the mode, and the verdict when the frame was
// disproved.
inline uint8_t packSnapshotMode(uint8_t mode, bool disproved, const float axis[3])
{
    uint8_t b = (uint8_t)(mode & SNAP_B2R_MODE_MASK);
    const uint8_t code = disproved ? axisCode(axis) : 0;
    if (code != 0)
        b = (uint8_t)(b | SNAP_B2R_DISPROVED | (uint8_t)(code << SNAP_B2R_AXIS_SHIFT));
    return b;
}

// The mode a restore applies.
inline uint8_t snapshotMode(uint8_t b2r_mode) { return (uint8_t)(b2r_mode & SNAP_B2R_MODE_MASK); }

// The restore's verdict: true, with the axis, when the snapshot says the boost
// thrust axis disproved its frame.  Never on a manual code (a manual frame is
// flown as set), and never from an older writer's byte (high bits 0).
inline bool restoredVerdict(uint8_t b2r_mode, float axis[3])
{
    if (!(b2r_mode & SNAP_B2R_DISPROVED)) return false;
    if (snapshotMode(b2r_mode) == ORIENT_MODE_MANUAL) return false;
    return axisFromCode((uint8_t)((b2r_mode & SNAP_B2R_AXIS_MASK) >> SNAP_B2R_AXIS_SHIFT), axis);
}

} // namespace ThrustAxisPolicy
