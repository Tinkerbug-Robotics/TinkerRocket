#ifndef TR_MAG_CALIBRATOR_H
#define TR_MAG_CALIBRATOR_H

// TR_MagCalibrator — hard-iron magnetometer calibration state machine.
//
// Issue #96.  Tumble-based sphere fit: the user rotates the rocket through
// all orientations while the FC accumulates raw mag samples; the
// calibrator solves a linear least-squares sphere fit to recover the
// hard-iron offset (cx, cy, cz) and local field magnitude R.
//
// Math: a perfect magnetometer in a uniform Earth field traces a sphere
// of radius |B_earth| (~50 µT) centered at the origin as the chip rotates.
// A hard-iron source (magnetized component on the PCB, steel hardware)
// shifts that sphere away from the origin; the offset is the new center.
// Soft-iron distortion would warp the sphere into an ellipsoid — out of
// scope for v1, see issue #96 follow-ups.
//
// Linear-LSQ form:
//   (x − cx)² + (y − cy)² + (z − cz)² = R²
//   2x·cx + 2y·cy + 2z·cz + k = x² + y² + z²       where k = R² − (cx² + cy² + cz²)
// → 4×4 normal equations; hand-rolled Gaussian elimination keeps the
// component dependency-free (no Eigen pull-in).
//
// Coverage: each sample's *direction* is binned into a 3×3×3 cube grid
// (axis-aligned thresholds at ±0.4 of the unit vector).  The (0,0,0)
// center cell is unreachable for a unit vector, leaving 26 wedges; a
// well-tumbled capture lights up ≥ 18 of them (issue #96 gate).
//
// Units: samples come in as raw int16 LSB counts (IIS2MDC: 0.15 µT/LSB).
// The fit is computed in raw counts and converted to µT only for the
// review/reject decisions.  This keeps the offset programmable directly
// into IIS2MDC OFFSET_X/Y/Z without scaling round-trips.

#include <compat.h>
#include <RocketComputerTypes.h>
#include <stdint.h>

class MagCalibrator
{
public:
    enum class State : uint8_t {
        IDLE      = 0,   // not running
        SAMPLING  = 1,   // accumulating samples; waiting for cap or user-stop
        REVIEW    = 2,   // sample cap hit, fit computed, awaiting user accept/retry/abort
        APPLIED   = 3,   // user accepted + post-accept verify passed; one-shot terminal
        ABORTED   = 4,   // user aborted or fit rejected during sampling-end → ready for IDLE
        // #206: post-accept verification pass.  Entered from REVIEW via
        // accept() — the FC has just programmed the chip's OFFSET regs
        // with the fit but hasn't persisted NVS yet.  Mag samples landing
        // in this state update min/max |B| accumulators; the FC drives a
        // ~5 s window and then calls evaluateVerify() to commit (→ APPLIED
        // with NVS write) or reject (→ REVIEW + MAG_CAL_REJECT_VERIFY_FAILED).
        VERIFYING = 5
    };

    MagCalibrator();

    // Begin sampling.  Clears prior buffer + coverage state.  Safe to call
    // from any internal state — semantically identical to abort() then start().
    void start();

    // User-requested abort.  Drops samples, transitions to ABORTED so the
    // FC can publish one final status frame before going IDLE.
    void abort();

    // User-requested retry from REVIEW.  Equivalent to start() — clears
    // samples and goes back to SAMPLING.
    void retry();

    // User-accepted fit.  Returns false (and stays in REVIEW) if no fit
    // is currently available (e.g. called before sample cap).  On success
    // transitions to VERIFYING (issue #206) — the caller is expected to
    // program the chip's OFFSET regs, feed verify samples via addSample,
    // and after a target dwell call evaluateVerify() to commit or reject.
    // The caller should pull the fit via getResult() before changing
    // OFFSET regs; the offsets remain valid throughout VERIFYING.
    bool accept();

    // #206: Evaluate the post-accept rotation.  Call after at least
    // MAG_CAL_VERIFY_MIN_SAMPLES samples have flowed through addSample
    // and the user has signalled they're done rotating.  Returns true
    // on pass — state → APPLIED and the caller persists NVS.  Returns
    // false on fail — state → REVIEW with one of the verify reject
    // sub-codes set, and the caller is responsible for zeroing the
    // chip's OFFSET regs (or restoring prior on a full abort).  No-op
    // outside VERIFYING; returns false in that case.  Pulls the
    // observed |B| worst-case into worst_uT for the rejected-frame
    // telemetry so iOS can show the user what went wrong.
    bool evaluateVerify(float& worst_uT);

    // #148: Clear the verify min/max/coverage accumulators without
    // leaving VERIFYING.  Lets the user tap "Retry verification" and
    // start a fresh rotation pass without going all the way back to
    // SAMPLING (which would invalidate the proposed fit).  No-op
    // outside VERIFYING.
    void resetVerify();

    // Reset to IDLE.  Call after consuming an APPLIED or ABORTED state.
    void clear();

    // Feed one raw mag sample (int16 LSB counts, sensor frame — same
    // units the IIS2MDC OFFSET registers consume).  No-op outside
    // SAMPLING.  Always returns false: the ring-buffer-per-wedge model
    // never has a "buffer full" pulse — the 5 Hz status cadence handles
    // iOS updates.  Each sample lands in the bucket for the rocket's
    // current accel-direction wedge (set via setLiveAccel), so samples
    // from one orientation can't crowd out samples from another.
    // Sampling NEVER auto-transitions to REVIEW — that's user-driven
    // via computeFit().
    bool addSample(int16_t x, int16_t y, int16_t z);

    // Set the latest low-g accelerometer reading (raw int16 counts,
    // body frame).  Used to bucket subsequent mag samples by physical
    // orientation: each call to addSample lands its sample in the
    // bucket for the wedge of (accel_x_, accel_y_, accel_z_).  Should
    // be called from the FC's IMU sample path; mag samples that
    // arrive before any accel reading are dropped (otherwise they'd
    // all pile into wedge 0).
    void setLiveAccel(int16_t ax, int16_t ay, int16_t az);

    // Run the sphere fit on the current sample buffer and transition to
    // REVIEW.  Returns false (and stays in SAMPLING) if fewer than
    // MAG_CAL_MIN_SAMPLES have landed.  Decoupling fit from buffer-fill
    // lets the iOS UI offer an explicit "Compute Fit" button: the user
    // tumbles as long as they want, then decides when to commit.
    bool computeFit();

    State getState() const { return state_; }

    // Live progress — meaningful in SAMPLING/REVIEW.  inst_field_uT is the
    // |B| of the most recent sample (post-offset-subtract if a prior fit
    // was applied; pre-offset on a fresh capture).
    void getProgress(uint16_t& sample_count,
                     uint8_t&  coverage_bins,
                     float&    inst_field_uT) const;

    // Final fit — meaningful only in REVIEW or APPLIED.  Caller checks
    // reject_code: 0 = ready to accept, non-zero = retry recommended.
    // R_uT is the fitted Earth-field magnitude in µT.  residual_uT is the
    // RMS deviation of samples from the fitted sphere, in µT.
    void getResult(int16_t& cx, int16_t& cy, int16_t& cz,
                   float& R_uT, float& residual_uT,
                   uint8_t& reject_code) const;

    // Convenience: pack the current state into a MagCalStatusData ready
    // for I2S TX.  time_us must be supplied by the caller (FC's monotonic
    // clock).  Always returns a valid frame regardless of state.
    void buildStatusFrame(uint32_t time_us, MagCalStatusData& out) const;

private:
    // Per-accel-wedge sample bucketing.  The flat ring buffer (older
    // design) lost diverse samples when the user lingered in one
    // orientation — newest samples blindly overwrote everything.  Now
    // each of the 32 (truncated-icosahedron / soccer-ball) accel
    // wedges has its own SAMPLES_PER_WEDGE ring buffer, so samples
    // from one orientation can only displace other samples from the
    // SAME orientation.  Memory: 32 × 100 × 3 × int16 ≈ 19 KiB,
    // comfortably small on the ESP32-P4.
    //
    // Tessellation (#148): 12 icosahedron vertices (pentagonal cell
    // centers) + 20 face centroids (hexagonal cell centers) on the
    // unit sphere → 32 Voronoi cells, all within ±5% of mean area.
    // Replaces the legacy 3³-1 = 26 cube-aligned wedges, which had
    // face-cells ~6× smaller than corner-cells and looked uneven in
    // the iOS sphere visualization.  See directionWedge() impl for
    // the cell-center table.
    static constexpr uint8_t  NUM_ACCEL_WEDGES   = 32;
    static constexpr uint8_t  SAMPLES_PER_WEDGE  = 100;
    static constexpr uint16_t MAX_TOTAL_SAMPLES  =
        (uint16_t)NUM_ACCEL_WEDGES * SAMPLES_PER_WEDGE;
    // Exposed for callers (e.g. iOS display caps); matches the old
    // MAG_CAL_MAX_SAMPLES contract.
    static constexpr uint16_t MAX_SAMPLES = MAX_TOTAL_SAMPLES;

    int16_t samples_x_[MAX_TOTAL_SAMPLES];
    int16_t samples_y_[MAX_TOTAL_SAMPLES];
    int16_t samples_z_[MAX_TOTAL_SAMPLES];
    uint8_t wedge_count_[NUM_ACCEL_WEDGES];  // 0..SAMPLES_PER_WEDGE per wedge
    uint8_t wedge_write_[NUM_ACCEL_WEDGES];  // ring write index per wedge
    uint16_t n_samples_;                      // sum of wedge_count_, kept in sync

    // Latest accel reading, used to pick the wedge bucket for each
    // incoming mag sample.  accel_valid_ stays false until the first
    // setLiveAccel call so we don't pile mag samples into wedge 0
    // before any orientation info has arrived.
    int16_t accel_x_, accel_y_, accel_z_;
    bool    accel_valid_;

    // 3³ - 1 = 26 wedges (the all-center cell is unreachable for unit
    // vectors).  Bit i set means wedge i has at least one sample.
    uint32_t coverage_mask_;

    // Most recent sample (for inst_field_uT progress reporting).
    int16_t last_x_, last_y_, last_z_;

    // Fit results in raw LSB counts.  R_lsb_ stays in raw units to dodge
    // a scaling round-trip when programming OFFSET_X/Y/Z.  R is what we
    // gate on, so cache it as float µT for cheap comparison.
    int16_t fit_cx_, fit_cy_, fit_cz_;
    float   fit_R_uT_;
    float   fit_residual_uT_;
    uint8_t fit_reject_code_;
    bool    fit_valid_;

    State state_;

    // #206 post-accept verification accumulators.  Reset on entering
    // VERIFYING via accept(); fed by addSample() while in VERIFYING.
    // verify_min_uT_ / verify_max_uT_ are tracked in µT (post-offset
    // chip subtract is already in raw counts, so the verify path
    // multiplies by IIS2MDC_LSB_TO_uT once per sample).
    // verify_coverage_mask_ is a parallel mask to coverage_mask_ that
    // only sees post-accept rotation — required to enforce that the
    // user actually rotated the rocket during verify and didn't just
    // hold it still.  verify_n_samples_ counts samples that arrived
    // while VERIFYING (not the sphere-fit buffer).
    float    verify_min_uT_, verify_max_uT_;
    uint16_t verify_n_samples_;
    uint32_t verify_coverage_mask_;

    // Map a unit-vector direction to a wedge index 0..25.  Returns
    // 26 (= invalid) if the input is the zero vector.
    static uint8_t directionWedge(int16_t x, int16_t y, int16_t z);

    // Run the sphere fit on the current sample buffer.  Sets fit_*.  No-op
    // if n_samples_ < MAG_CAL_MIN_SAMPLES.
    void runFit();
};

#endif // TR_MAG_CALIBRATOR_H
