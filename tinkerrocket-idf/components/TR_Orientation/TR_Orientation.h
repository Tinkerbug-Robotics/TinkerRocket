#ifndef TR_ORIENTATION_H
#define TR_ORIENTATION_H

// Board→rocket mounting orientation utilities.
//
// The SensorConverter maps sensor frame → board frame with a fixed
// per-chip Z rotation (PCB layout fact).  This component describes the
// second hop: how the *board* sits in the *rocket* (an airframe
// integration fact).  Board frame and rocket frame are both FLU
// (+X forward/nose, +Y left, +Z up); when the board is mounted with its
// +X toward the nose the mapping is identity and nothing changes.
//
// Discrete orientations are encoded in one byte:
//   code = nose_sel * 4 + clock        (24 proper rotations, 0 = identity)
//   nose_sel: which BOARD axis points toward the rocket NOSE
//     0=+X  1=-X  2=+Y  3=-Y  4=+Z  5=-Z
//   clock: extra quarter-turns (0..3, CCW looking down the nose from
//     outside, i.e. right-hand rule about rocket +X) applied after the
//     nose alignment.  clock matters only for roll-referenced features
//     (roll control / canard mixing); recovery logic is roll-agnostic.
//
// For each nose_sel, clock=0 is the shortest-arc rotation from the board
// axis to the nose (for -X, where the arc is ambiguous, the convention is
// a 180° turn about board +Z).  This matches what a gravity-vector
// auto-detect naturally produces, so snapped auto codes and manual codes
// agree.
//
// Matrices are row-major 3x3, v_rocket = R * v_board.

#include <stdint.h>

// Identity mounting: board +X toward the nose, no clocking.
static constexpr uint8_t ORIENT_CODE_IDENTITY = 0;
static constexpr uint8_t ORIENT_CODE_COUNT    = 24;

// How the active board→rocket rotation was determined.  Carried beside
// the code/quaternion on the wire and in the flight-settings snapshot.
static constexpr uint8_t ORIENT_MODE_DEFAULT    = 0;  // identity, nothing configured
static constexpr uint8_t ORIENT_MODE_MANUAL     = 1;  // user/config supplied code
static constexpr uint8_t ORIENT_MODE_AUTO_SNAP  = 2;  // pad-gravity detect, snapped to axis
static constexpr uint8_t ORIENT_MODE_AUTO_EXACT = 3;  // pad-gravity detect, exact vector (snap tolerance exceeded)

// Scale used when quaternions ride the wire as int16 (matches the
// NonSensorData quaternion convention).
static constexpr float ORIENT_QUAT_WIRE_SCALE = 10000.0f;

// code → rotation matrix (row-major, v_rocket = R * v_board).
// Invalid codes (>= ORIENT_CODE_COUNT) yield identity.
void orientCodeToMatrix(uint8_t code, float R[9]);

// Exact inverse of orientCodeToMatrix: matches R against the 24 discrete
// rotations (tolerance ~1e-3 per entry).  Returns false (code untouched)
// if R is not one of them — e.g. an AUTO_EXACT rotation.
bool orientMatrixToCode(const float R[9], uint8_t &code);

// code → unit quaternion (scalar-first, board→rocket).
void orientCodeToQuat(uint8_t code, float q[4]);

// Unit quaternion (scalar-first) → rotation matrix (row-major).
void orientQuatToMatrix(const float q[4], float R[9]);

// Rotation matrix → unit quaternion (scalar-first, q0 >= 0).
void orientMatrixToQuat(const float R[9], float q[4]);

// Snap a board-frame "toward the nose" vector (e.g. averaged specific
// force on a vertical pad) to the nearest ±axis.  Returns the orientation
// code (clock = 0) and, optionally, the residual angle between the vector
// and the snapped axis in degrees.  Zero-length input returns identity
// with residual 0.
uint8_t orientNearestNoseCode(const float v_board[3], float *residual_deg);

// Short human-readable name for a code: "+X", "-Z r90", ...  Returns a
// pointer to a static string ("?" for invalid codes).
const char *orientCodeName(uint8_t code);

// Pad attitude initialization: body-to-NED quaternion from a stationary
// FRD specific-force measurement plus a known pad heading.  This is the
// EKF init previously inlined in flight_computer main.cpp — pitch from
// the X component, roll from Y/Z (skipped within 10° of vertical where it
// is ill-conditioned), yaw from the supplied heading.  Works for any
// attitude, which is what makes the EKF init mounting-agnostic once the
// board→rocket rotation is applied upstream.
void quatFromAccelHeading(float acc_x_frd, float acc_y_frd, float acc_z_frd,
                          float heading_rad, float q[4]);

#endif // TR_ORIENTATION_H
