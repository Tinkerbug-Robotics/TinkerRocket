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

// Exact (non-snapped) board→rocket rotation from a measured nose vector:
// the shortest-arc rotation taking v̂ to +X.  Used when the pad-gravity
// estimate is further than the snap tolerance from every axis (board
// mounted off-orthogonal).  For axis-aligned v this reproduces the
// canonical clock=0 matrices, including the 180°-about-+Z convention for
// -X.  Returns false (R = identity) for a zero-length input.
bool orientMatrixFromNoseVector(const float v_board[3], float R[9]);

// ---------------------------------------------------------------------------
// Pad-gravity orientation estimator.
//
// Continuously averages the specific-force direction while the rocket sits
// still before launch.  On a (near-)vertical pad that direction IS the nose
// direction, so it reveals how the board is mounted.  The caller feeds
// converted ROCKET-frame samples (post board→rocket rotation) and, on each
// completed window, decides whether the active rotation needs updating —
// if the mounting is already correct the estimate simply confirms +X.
//
// Stationarity gating: every gyro axis under gyro_quiet_dps AND |‖a‖−g|
// under acc_tol_ms2; any violation restarts the window, so handling,
// wind-rock, and transport never contribute samples.  Duplicate timestamps
// (caller polls faster than the IMU updates) are ignored.
class OrientationEstimator
{
public:
    // Thresholds are members (not constructor params) so tests and future
    // config plumbing can tune them; defaults match the landing detector's
    // notion of "still" with margin for pad wind.
    float    gyro_quiet_dps = 3.0f;      // per-axis |gyro| ceiling
    float    acc_tol_ms2    = 2.0f;      // | ‖a‖ − g | ceiling (~0.2 g)
    uint32_t window_us      = 2000000;   // averaging window
    uint32_t min_samples    = 200;       // floor on samples per window

    void reset();

    // Feed one rocket-frame SI sample (accel m/s², gyro dps).  Returns true
    // when this sample completed a window (a fresh estimate is available).
    bool update(uint32_t t_us, float ax, float ay, float az,
                float gx, float gy, float gz);

    // Fetch the latest completed estimate (unit "up/nose" vector in rocket
    // frame) and clear the fresh flag.  False if no unread estimate.
    bool takeEstimate(float up_rocket[3]);

private:
    double   sum_[3]        = {0.0, 0.0, 0.0};
    uint32_t count_         = 0;
    uint32_t window_start_us_ = 0;
    uint32_t last_t_us_     = 0;
    bool     accumulating_  = false;
    bool     fresh_         = false;
    float    est_[3]        = {1.0f, 0.0f, 0.0f};
};

// ---------------------------------------------------------------------------
// Thrust-axis cross-check.
//
// During the first instants of powered flight the specific force is
// dominated by thrust, which acts along the rocket axis regardless of rail
// tilt.  Averaging the rocket-frame accel direction right after launch
// detection and comparing it to +X validates the latched orientation: a
// large angle means the pad-gravity estimate (or a manual setting) was
// wrong — e.g. the rocket sat non-vertical at the final stationary window.
// The result is a flag, not a correction: re-orienting mid-boost would
// yank the EKF and control frames at the worst possible moment.
class ThrustAxisCheck
{
public:
    float    min_acc_ms2 = 20.0f;   // only count clearly-thrusting samples
    uint32_t window_us   = 150000;  // averaging window after start()

    void start(uint32_t t_us);
    // Feed rocket-frame accel.  Returns true once, when the window closes.
    bool update(uint32_t t_us, float ax, float ay, float az);

    bool  done() const  { return done_; }
    bool  valid() const { return valid_; }      // enough samples to judge
    float angleFromNoseDeg() const { return angle_deg_; }

private:
    double   sum_[3]   = {0.0, 0.0, 0.0};
    uint32_t count_    = 0;
    uint32_t start_us_ = 0;
    uint32_t last_t_us_ = 0;
    bool     running_  = false;
    bool     done_     = false;
    bool     valid_    = false;
    float    angle_deg_ = 0.0f;
};

#endif // TR_ORIENTATION_H
