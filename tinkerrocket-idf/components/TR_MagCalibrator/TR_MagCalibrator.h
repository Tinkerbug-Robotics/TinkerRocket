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
        APPLIED   = 3,   // user accepted; one-shot terminal so the FC can latch + transition
        ABORTED   = 4    // user aborted or fit rejected during sampling-end → ready for IDLE
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

    // User-accepted fit.  Returns false (and stays in REVIEW) if no fit is
    // currently available (e.g. called before sample cap).  On success
    // transitions to APPLIED; the caller should pull the fit via
    // getResult() and then call clear() before the next start().
    bool accept();

    // Reset to IDLE.  Call after consuming an APPLIED or ABORTED state.
    void clear();

    // Feed one raw sample (int16 LSB counts, sensor frame — same units the
    // IIS2MDC OFFSET registers consume).  No-op outside SAMPLING.  Returns
    // true if the sample was the one that filled the buffer (the caller
    // can use this as a "buffer full, stop pacing samples" hint, though
    // additional samples silently no-op once full).  Sampling NEVER
    // auto-transitions to REVIEW — that's user-driven via computeFit().
    bool addSample(int16_t x, int16_t y, int16_t z);

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
    // Tunables — see MAG_CAL_* constants in RocketComputerTypes.h.
    static constexpr uint16_t MAX_SAMPLES = MAG_CAL_MAX_SAMPLES;

    // Sample storage.  3 × int16 × 1024 = 6 KiB on the FC heap.
    int16_t samples_x_[MAX_SAMPLES];
    int16_t samples_y_[MAX_SAMPLES];
    int16_t samples_z_[MAX_SAMPLES];
    uint16_t n_samples_;

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

    // Map a unit-vector direction to a wedge index 0..25.  Returns
    // 26 (= invalid) if the input is the zero vector.
    static uint8_t directionWedge(int16_t x, int16_t y, int16_t z);

    // Run the sphere fit on the current sample buffer.  Sets fit_*.  No-op
    // if n_samples_ < MAG_CAL_MIN_SAMPLES.
    void runFit();
};

#endif // TR_MAG_CALIBRATOR_H
