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

#include <cmath>
#include <cstdint>

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

} // namespace ThrustAxisPolicy
