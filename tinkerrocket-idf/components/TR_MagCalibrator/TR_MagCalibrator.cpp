#include "TR_MagCalibrator.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_log.h"
static const char* TAG = "MAGCAL";
#endif

// IIS2MDC native sensitivity: 0.15 µT/LSB (datasheet 9.13).  The
// calibrator works in raw int16 counts so the fit can be programmed
// directly into OFFSET_X/Y/Z without scaling round-trips; this constant
// only shows up where we need µT for the R-band gate or BLE telemetry.
static constexpr float IIS2MDC_LSB_TO_uT = 0.15f;

// Gaussian-elimination 4×4 solver (in-place; A is destroyed).  Returns
// false if the matrix is singular (degenerate sample geometry, e.g. all
// samples on a great circle — happens if the user only spins around one
// axis).  Partial pivoting is enough; the system is symmetric PSD when
// well-conditioned.
static bool solve4x4(double A[4][4], double b[4], double x[4])
{
    for (int col = 0; col < 4; col++)
    {
        // Partial pivot
        int piv = col;
        double piv_abs = fabs(A[col][col]);
        for (int row = col + 1; row < 4; row++)
        {
            double a = fabs(A[row][col]);
            if (a > piv_abs) { piv = row; piv_abs = a; }
        }
        if (piv_abs < 1e-9) return false;
        if (piv != col)
        {
            for (int k = 0; k < 4; k++) { double t = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = t; }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        // Eliminate below
        for (int row = col + 1; row < 4; row++)
        {
            const double f = A[row][col] / A[col][col];
            for (int k = col; k < 4; k++) A[row][k] -= f * A[col][k];
            b[row] -= f * b[col];
        }
    }
    // Back-substitute
    for (int row = 3; row >= 0; row--)
    {
        double s = b[row];
        for (int k = row + 1; k < 4; k++) s -= A[row][k] * x[k];
        x[row] = s / A[row][row];
    }
    return true;
}

MagCalibrator::MagCalibrator()
    : n_samples_(0),
      coverage_mask_(0),
      last_x_(0), last_y_(0), last_z_(0),
      fit_cx_(0), fit_cy_(0), fit_cz_(0),
      fit_R_uT_(0.0f),
      fit_residual_uT_(0.0f),
      fit_reject_code_(MAG_CAL_OK),
      fit_valid_(false),
      accel_x_(0), accel_y_(0), accel_z_(0),
      accel_valid_(false),
      state_(State::IDLE),
      verify_min_uT_(0.0f), verify_max_uT_(0.0f),
      verify_n_samples_(0),
      verify_coverage_mask_(0)
{
    memset(wedge_count_, 0, sizeof(wedge_count_));
    memset(wedge_write_, 0, sizeof(wedge_write_));
}

void MagCalibrator::start()
{
    n_samples_ = 0;
    memset(wedge_count_, 0, sizeof(wedge_count_));
    memset(wedge_write_, 0, sizeof(wedge_write_));
    coverage_mask_ = 0;
    last_x_ = last_y_ = last_z_ = 0;
    // accel_valid_ stays as-is: the IMU is sampling regardless of cal
    // state, so accel readings from before Start are still fresh.
    fit_valid_ = false;
    fit_cx_ = fit_cy_ = fit_cz_ = 0;
    fit_R_uT_ = 0.0f;
    fit_residual_uT_ = 0.0f;
    fit_reject_code_ = MAG_CAL_OK;
    state_ = State::SAMPLING;
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "start: SAMPLING, target=%u samples", (unsigned)MAX_SAMPLES);
#endif
}

void MagCalibrator::abort()
{
    state_ = State::ABORTED;
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "abort: ABORTED (n=%u)", (unsigned)n_samples_);
#endif
}

void MagCalibrator::retry()
{
    start();
}

bool MagCalibrator::accept()
{
    if (state_ != State::REVIEW || !fit_valid_) return false;
    // #206: REVIEW → VERIFYING.  Caller programs OFFSET regs immediately
    // after this returns, so subsequent addSample calls see post-offset
    // |B|.  Reset the verify accumulators so prior runs (re-cal then
    // verify-fail then re-cal) don't carry state forward.
    verify_min_uT_ = 0.0f;
    verify_max_uT_ = 0.0f;
    verify_n_samples_ = 0;
    verify_coverage_mask_ = 0;
    state_ = State::VERIFYING;
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "accept: VERIFYING (cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT)",
             (int)fit_cx_, (int)fit_cy_, (int)fit_cz_,
             (double)fit_R_uT_, (double)fit_residual_uT_);
#endif
    return true;
}

bool MagCalibrator::evaluateVerify(float& worst_uT)
{
    worst_uT = 0.0f;
    if (state_ != State::VERIFYING) return false;

    const uint8_t cov_bins = (uint8_t)__builtin_popcount(verify_coverage_mask_);
    const float range_uT = verify_max_uT_ - verify_min_uT_;

    // Sample-count floor first — without enough samples min/max are
    // meaningless and we'd flap on noise.
    if (verify_n_samples_ < MAG_CAL_VERIFY_MIN_SAMPLES) {
        // Treat as fail-with-rotate-more so the user gets a clear retry.
        fit_reject_code_ = MAG_CAL_REJECT_VERIFY_FAILED;
        worst_uT = verify_max_uT_;
        state_ = State::REVIEW;
#ifdef ESP_PLATFORM
        ESP_LOGW(TAG, "verify FAIL (insufficient samples): n=%u (need %u)",
                 (unsigned)verify_n_samples_, (unsigned)MAG_CAL_VERIFY_MIN_SAMPLES);
#endif
        return false;
    }

    // Coverage check — the user must rotate the rocket through enough
    // orientations during verify.  Without rotation a wrong cal would
    // trivially pass the |B| gate (rocket is stationary; |B| varies only
    // by sensor noise).  Threshold is lower than SAMPLING because the
    // verify window is short — we just need *some* rotation diversity.
    if (cov_bins < MAG_CAL_VERIFY_MIN_COVERAGE_BINS) {
        fit_reject_code_ = MAG_CAL_REJECT_VERIFY_FAILED;
        worst_uT = verify_max_uT_;
        state_ = State::REVIEW;
#ifdef ESP_PLATFORM
        ESP_LOGW(TAG, "verify FAIL (low coverage): bins=%u (need %u)",
                 (unsigned)cov_bins, (unsigned)MAG_CAL_VERIFY_MIN_COVERAGE_BINS);
#endif
        return false;
    }

    // Magnitude band gate.
    if (verify_max_uT_ > MAG_CAL_VERIFY_MAX_UT) {
        fit_reject_code_ = MAG_CAL_REJECT_VERIFY_FAILED;
        worst_uT = verify_max_uT_;
        state_ = State::REVIEW;
#ifdef ESP_PLATFORM
        ESP_LOGW(TAG, "verify FAIL (|B| too high): max=%.1fµT > %.1f",
                 (double)verify_max_uT_, (double)MAG_CAL_VERIFY_MAX_UT);
#endif
        return false;
    }
    if (verify_min_uT_ < MAG_CAL_VERIFY_MIN_UT) {
        fit_reject_code_ = MAG_CAL_REJECT_VERIFY_FAILED;
        worst_uT = verify_min_uT_;
        state_ = State::REVIEW;
#ifdef ESP_PLATFORM
        ESP_LOGW(TAG, "verify FAIL (|B| too low): min=%.1fµT < %.1f",
                 (double)verify_min_uT_, (double)MAG_CAL_VERIFY_MIN_UT);
#endif
        return false;
    }
    // Range (max-min) gate — catches a residual hard-iron that keeps
    // |B| in the band on average but swings substantially with rotation.
    if (range_uT > MAG_CAL_VERIFY_RANGE_UT) {
        fit_reject_code_ = MAG_CAL_REJECT_VERIFY_FAILED;
        // Report whichever extreme is further from the fitted R as the
        // "worst" — that's the more diagnostic value for the user.
        worst_uT = ((fit_R_uT_ - verify_min_uT_) > (verify_max_uT_ - fit_R_uT_))
                   ? verify_min_uT_ : verify_max_uT_;
        state_ = State::REVIEW;
#ifdef ESP_PLATFORM
        ESP_LOGW(TAG, "verify FAIL (|B| range too wide): range=%.1fµT > %.1f (min=%.1f max=%.1f)",
                 (double)range_uT, (double)MAG_CAL_VERIFY_RANGE_UT,
                 (double)verify_min_uT_, (double)verify_max_uT_);
#endif
        return false;
    }

    // Pass — APPLIED is the terminal good state; FC writes NVS next.
    state_ = State::APPLIED;
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "verify PASS: |B| min=%.1f max=%.1f range=%.1f (n=%u, cov=%u/26)",
             (double)verify_min_uT_, (double)verify_max_uT_, (double)range_uT,
             (unsigned)verify_n_samples_, (unsigned)cov_bins);
#endif
    return true;
}

void MagCalibrator::clear()
{
    state_ = State::IDLE;
}

void MagCalibrator::setLiveAccel(int16_t ax, int16_t ay, int16_t az)
{
    accel_x_ = ax;
    accel_y_ = ay;
    accel_z_ = az;
    accel_valid_ = true;
}

bool MagCalibrator::addSample(int16_t x, int16_t y, int16_t z)
{
    // #206: during VERIFYING the chip's OFFSET regs are already
    // programmed, so x/y/z arriving here are post-offset.  Track
    // |B| min/max plus rotation coverage; no per-orientation
    // bucketing (the sphere-fit buffer is frozen at the REVIEW fit).
    if (state_ == State::VERIFYING) {
        last_x_ = x;
        last_y_ = y;
        last_z_ = z;

        const float fx = (float)x * IIS2MDC_LSB_TO_uT;
        const float fy = (float)y * IIS2MDC_LSB_TO_uT;
        const float fz = (float)z * IIS2MDC_LSB_TO_uT;
        const float B = sqrtf(fx*fx + fy*fy + fz*fz);

        if (verify_n_samples_ == 0) {
            verify_min_uT_ = B;
            verify_max_uT_ = B;
        } else {
            if (B < verify_min_uT_) verify_min_uT_ = B;
            if (B > verify_max_uT_) verify_max_uT_ = B;
        }
        verify_n_samples_++;

        // Coverage from accel wedge — same logic as SAMPLING.  Without
        // a fresh accel reading we just skip the coverage update; the
        // |B| min/max still tick so a stationary verify can still hit
        // the sample floor and fail-on-coverage rather than silently
        // accumulating samples in a single wedge.
        if (accel_valid_) {
            const uint8_t aw = directionWedge(accel_x_, accel_y_, accel_z_);
            if (aw < NUM_ACCEL_WEDGES) verify_coverage_mask_ |= (1u << aw);
        }
        return false;
    }

    if (state_ != State::SAMPLING) return false;

    // Live vector + coverage update regardless of bucket state — the
    // iOS direction-feedback UI keeps ticking and the coverage mask
    // tracks which accel-direction wedges have been visited so far.
    last_x_ = x;
    last_y_ = y;
    last_z_ = z;

    // Without a recent accel reading we can't bin the sample by
    // orientation, so drop it rather than piling everything into a
    // single bucket.  Should only happen during the brief startup
    // window before the first IMU sample lands.
    if (!accel_valid_) return false;

    const uint8_t accel_wedge = directionWedge(accel_x_, accel_y_, accel_z_);
    if (accel_wedge >= NUM_ACCEL_WEDGES) return false;

    // Update the accel-driven coverage mask up-front so the iOS UI
    // sees the new wedge as soon as it's been visited, even if its
    // slot ring buffer is full and an oldest sample gets evicted.
    coverage_mask_ |= (1u << accel_wedge);

    // Ring buffer inside the wedge bucket.  Up to SAMPLES_PER_WEDGE
    // samples are kept per accel-wedge; when full, newest overwrites
    // oldest WITHIN THAT WEDGE.  Samples from other orientations are
    // never disturbed, so a user lingering in one position can't
    // wipe out the diversity gathered earlier.
    const uint16_t base = (uint16_t)accel_wedge * SAMPLES_PER_WEDGE;
    const uint16_t slot = base + wedge_write_[accel_wedge];
    samples_x_[slot] = x;
    samples_y_[slot] = y;
    samples_z_[slot] = z;
    wedge_write_[accel_wedge] = (uint8_t)((wedge_write_[accel_wedge] + 1) % SAMPLES_PER_WEDGE);
    if (wedge_count_[accel_wedge] < SAMPLES_PER_WEDGE) {
        wedge_count_[accel_wedge]++;
        n_samples_++;  // saturates at NUM_ACCEL_WEDGES * SAMPLES_PER_WEDGE
    }

    // Sampling never "completes" — the 5 Hz status cadence in the FC
    // main loop keeps the iOS UI updated.  No "buffer just filled" pulse.
    return false;
}

bool MagCalibrator::computeFit()
{
    if (state_ != State::SAMPLING) return false;
    if (n_samples_ < MAG_CAL_MIN_SAMPLES) return false;
    runFit();
    state_ = State::REVIEW;
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "user-triggered fit complete: cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT cov=%u/26 reject=%u (n=%u)",
             (int)fit_cx_, (int)fit_cy_, (int)fit_cz_,
             (double)fit_R_uT_, (double)fit_residual_uT_,
             (unsigned)__builtin_popcount(coverage_mask_),
             (unsigned)fit_reject_code_,
             (unsigned)n_samples_);
#endif
    return true;
}

void MagCalibrator::getProgress(uint16_t& sample_count,
                                uint8_t&  coverage_bins,
                                float&    inst_field_uT) const
{
    sample_count = n_samples_;
    coverage_bins = (uint8_t)__builtin_popcount(coverage_mask_);
    // |B| of the most recent sample, in µT.  Note: this is pre-fit-apply
    // — the IIS2MDC chip's OFFSET registers haven't been programmed yet
    // during cal, so this includes the hard-iron contribution.  After
    // accept(), boot-time apply zeroes the offset out of the raw stream.
    const float fx = (float)last_x_ * IIS2MDC_LSB_TO_uT;
    const float fy = (float)last_y_ * IIS2MDC_LSB_TO_uT;
    const float fz = (float)last_z_ * IIS2MDC_LSB_TO_uT;
    inst_field_uT = sqrtf(fx*fx + fy*fy + fz*fz);
}

void MagCalibrator::getResult(int16_t& cx, int16_t& cy, int16_t& cz,
                              float& R_uT, float& residual_uT,
                              uint8_t& reject_code) const
{
    cx = fit_cx_;
    cy = fit_cy_;
    cz = fit_cz_;
    R_uT = fit_R_uT_;
    residual_uT = fit_residual_uT_;
    reject_code = fit_reject_code_;
}

void MagCalibrator::buildStatusFrame(uint32_t time_us, MagCalStatusData& out) const
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    switch (state_)
    {
        case State::IDLE:      out.sub_type = MAG_CAL_SUB_IDLE;      break;
        case State::SAMPLING:  out.sub_type = MAG_CAL_SUB_SAMPLING;  break;
        case State::REVIEW:    out.sub_type = MAG_CAL_SUB_REVIEW;    break;
        case State::APPLIED:   out.sub_type = MAG_CAL_SUB_APPLIED;   break;
        case State::ABORTED:   out.sub_type = MAG_CAL_SUB_ABORTED;   break;
        case State::VERIFYING: out.sub_type = MAG_CAL_SUB_VERIFYING; break;
    }

    // During VERIFYING the coverage / sample-count fields report the
    // verify-window progress so iOS can render a progress bar that
    // matches what the FC is actually gating on.  Outside VERIFYING
    // the sphere-fit's own counters are the right thing to show.
    if (state_ == State::VERIFYING) {
        out.coverage_bins = (uint8_t)__builtin_popcount(verify_coverage_mask_);
        out.coverage_mask = verify_coverage_mask_;
        out.sample_count  = verify_n_samples_;
    } else {
        out.coverage_bins = (uint8_t)__builtin_popcount(coverage_mask_);
        out.coverage_mask = coverage_mask_;
        out.sample_count  = n_samples_;
    }

    // Live raw vector for the iOS direction-feedback UI.  Raw LSB
    // matches the sphere-fit's working units so no conversion drift.
    out.inst_x_lsb = last_x_;
    out.inst_y_lsb = last_y_;
    out.inst_z_lsb = last_z_;

    // Instantaneous field magnitude (most recent sample).
    {
        const float fx = (float)last_x_ * IIS2MDC_LSB_TO_uT;
        const float fy = (float)last_y_ * IIS2MDC_LSB_TO_uT;
        const float fz = (float)last_z_ * IIS2MDC_LSB_TO_uT;
        const float mag_uT = sqrtf(fx*fx + fy*fy + fz*fz);
        const float mag_x10 = mag_uT * 10.0f;
        out.inst_field_uT_x10 = (mag_x10 < 0.0f) ? 0u
                              : (mag_x10 > 65535.0f) ? 65535u
                              : (uint16_t)mag_x10;
    }

    if (fit_valid_)
    {
        out.offset_x = fit_cx_;
        out.offset_y = fit_cy_;
        out.offset_z = fit_cz_;
        const float r_x10 = fit_R_uT_ * 10.0f;
        const float res_x10 = fit_residual_uT_ * 10.0f;
        out.field_R_uT_x10  = (r_x10 < 0.0f) ? 0u : (r_x10 > 65535.0f) ? 65535u : (uint16_t)r_x10;
        out.residual_uT_x10 = (res_x10 < 0.0f) ? 0u : (res_x10 > 65535.0f) ? 65535u : (uint16_t)res_x10;
        out.reject_code     = fit_reject_code_;
    }
}

// 32 cell centers on the unit sphere — truncated icosahedron / soccer-ball
// tessellation.  Indices 0–11 are icosahedron vertices (pentagonal cell
// centers); indices 12–31 are icosahedron face centroids (hexagonal cell
// centers).  All entries are unit vectors; closest pair is separated by
// ~37.4° on the sphere.  Voronoi cell areas are within ±5% of mean —
// pentagons ~96% of mean, hexagons ~103% (verified via Monte Carlo).
//
// Generated by tools/generate_cell_centers.py (see #148 commit message
// for the derivation).  φ = (1 + √5) / 2; icosa vertices are the cyclic
// permutations of (0, ±1, ±φ); face centroids = normalized mean of the
// three adjacent vertex coordinates.
static constexpr float CELL_X[32] = {
     0.00000000f, -0.52573111f, -0.85065081f,  0.00000000f, -0.52573111f, -0.85065081f,
     0.00000000f,  0.52573111f,  0.85065081f,  0.00000000f,  0.52573111f,  0.85065081f,
    -0.57735027f,  0.00000000f, -0.35682209f,  0.35682209f,  0.57735027f, -0.93417236f,
    -0.57735027f,  0.00000000f, -0.93417236f, -0.57735027f, -0.35682209f,  0.57735027f,
     0.35682209f, -0.57735027f,  0.00000000f,  0.00000000f,  0.57735027f,  0.93417236f,
     0.93417236f,  0.57735027f,
};
static constexpr float CELL_Y[32] = {
    -0.52573111f, -0.85065081f,  0.00000000f, -0.52573111f,  0.85065081f,  0.00000000f,
     0.52573111f, -0.85065081f,  0.00000000f,  0.52573111f,  0.85065081f,  0.00000000f,
    -0.57735027f, -0.93417236f,  0.00000000f,  0.00000000f, -0.57735027f, -0.35682209f,
    -0.57735027f, -0.93417236f,  0.35682209f,  0.57735027f,  0.00000000f, -0.57735027f,
     0.00000000f,  0.57735027f,  0.93417236f,  0.93417236f,  0.57735027f, -0.35682209f,
     0.35682209f,  0.57735027f,
};
static constexpr float CELL_Z[32] = {
    -0.85065081f,  0.00000000f, -0.52573111f,  0.85065081f,  0.00000000f,  0.52573111f,
    -0.85065081f,  0.00000000f, -0.52573111f,  0.85065081f,  0.00000000f,  0.52573111f,
    -0.57735027f, -0.35682209f, -0.93417236f, -0.93417236f, -0.57735027f,  0.00000000f,
     0.57735027f,  0.35682209f,  0.00000000f, -0.57735027f,  0.93417236f,  0.57735027f,
     0.93417236f,  0.57735027f, -0.35682209f,  0.35682209f, -0.57735027f,  0.00000000f,
     0.00000000f,  0.57735027f,
};

uint8_t MagCalibrator::directionWedge(int16_t x, int16_t y, int16_t z)
{
    // Reject the zero vector (sample exactly at origin — implausible from
    // a real sensor, but defends against a degenerate path).
    const double xx = (double)x * (double)x;
    const double yy = (double)y * (double)y;
    const double zz = (double)z * (double)z;
    const double r2 = xx + yy + zz;
    if (r2 <= 0.0) return NUM_ACCEL_WEDGES;  // sentinel = invalid

    const double r = sqrt(r2);
    const float ux = (float)((double)x / r);
    const float uy = (float)((double)y / r);
    const float uz = (float)((double)z / r);

    // Voronoi assignment: pick the cell center with maximum dot product
    // against the normalized sample direction.  32 dot products + max
    // is ~100 cycles on ESP32-P4, easily under the per-sample budget.
    float bestD = -2.0f;
    uint8_t bestI = 0;
    for (uint8_t i = 0; i < NUM_ACCEL_WEDGES; i++) {
        const float d = ux * CELL_X[i] + uy * CELL_Y[i] + uz * CELL_Z[i];
        if (d > bestD) { bestD = d; bestI = i; }
    }
    return bestI;
}

void MagCalibrator::runFit()
{
    fit_valid_ = false;
    fit_cx_ = fit_cy_ = fit_cz_ = 0;
    fit_R_uT_ = 0.0f;
    fit_residual_uT_ = 0.0f;
    fit_reject_code_ = MAG_CAL_OK;

    if (n_samples_ < MAG_CAL_MIN_SAMPLES) return;

    // All sample iteration walks the per-wedge bucket arrays.  Helper
    // lambda captures the "for each valid sample" pattern so the three
    // passes (centroid, normal eqs, residual) share the same indexing
    // logic and stay in sync if the storage layout changes.
    auto forEachSample = [this](auto&& fn) {
        for (uint8_t w = 0; w < NUM_ACCEL_WEDGES; w++) {
            const uint16_t base = (uint16_t)w * SAMPLES_PER_WEDGE;
            const uint8_t  count = wedge_count_[w];
            for (uint8_t i = 0; i < count; i++) {
                const uint16_t slot = base + i;
                fn(samples_x_[slot], samples_y_[slot], samples_z_[slot]);
            }
        }
    };

    // Accumulate sums in double.  Centroid-shifting before solving keeps
    // the cross-product sums small (cancel the bulk of the offset before
    // squaring), which preserves ~7+ digits of precision in the normal
    // equations even with raw counts in the ~1e4 range.
    double sx = 0.0, sy = 0.0, sz = 0.0;
    forEachSample([&](int16_t x, int16_t y, int16_t z) {
        sx += (double)x;
        sy += (double)y;
        sz += (double)z;
    });
    const double mx = sx / (double)n_samples_;
    const double my = sy / (double)n_samples_;
    const double mz = sz / (double)n_samples_;

    // Normal-equation accumulators on centered samples (xc = x - mx etc).
    // A row = [2·xc, 2·yc, 2·zc, 1]; b = xc² + yc² + zc².
    double sxx = 0.0, syy = 0.0, szz = 0.0;
    double sxy = 0.0, sxz = 0.0, syz = 0.0;
    double sx_c = 0.0, sy_c = 0.0, sz_c = 0.0;  // first-moments of centered samples (≈ 0 by construction)
    double sb = 0.0;
    double sxb = 0.0, syb = 0.0, szb = 0.0;

    forEachSample([&](int16_t x, int16_t y, int16_t z) {
        const double xc = (double)x - mx;
        const double yc = (double)y - my;
        const double zc = (double)z - mz;
        const double bi = xc*xc + yc*yc + zc*zc;

        sxx += xc*xc; syy += yc*yc; szz += zc*zc;
        sxy += xc*yc; sxz += xc*zc; syz += yc*zc;
        sx_c += xc; sy_c += yc; sz_c += zc;
        sb += bi;
        sxb += xc * bi;
        syb += yc * bi;
        szb += zc * bi;
    });

    // Build symmetric AᵀA (4×4) and Aᵀb (4×1) for the centered solve.
    double M[4][4] = {
        { 4*sxx, 4*sxy, 4*sxz, 2*sx_c },
        { 4*sxy, 4*syy, 4*syz, 2*sy_c },
        { 4*sxz, 4*syz, 4*szz, 2*sz_c },
        { 2*sx_c, 2*sy_c, 2*sz_c, (double)n_samples_ }
    };
    double rhs[4] = { 2*sxb, 2*syb, 2*szb, sb };
    double p[4] = { 0, 0, 0, 0 };

    if (!solve4x4(M, rhs, p))
    {
        // Singular (samples collinear / coplanar) — caller will flag low
        // coverage, but mark fit as bad explicitly so accept() refuses.
        fit_reject_code_ = MAG_CAL_REJECT_LOW_COVERAGE;
        return;
    }

    // p in centered frame: cx_c, cy_c, cz_c, k_c where
    //   (xc - cx_c)² + (yc - cy_c)² + (zc - cz_c)² = R²
    // Translate back: cx = cx_c + mx, etc.  R is the same in both frames.
    const double cx_c = p[0];
    const double cy_c = p[1];
    const double cz_c = p[2];
    const double k_c  = p[3];
    const double R2 = k_c + cx_c*cx_c + cy_c*cy_c + cz_c*cz_c;
    if (!(R2 > 0.0))
    {
        fit_reject_code_ = MAG_CAL_REJECT_LOW_COVERAGE;
        return;
    }
    const double R = sqrt(R2);

    const double cx = cx_c + mx;
    const double cy = cy_c + my;
    const double cz = cz_c + mz;

    // RMS residual in raw counts (then scaled to µT).
    double rss = 0.0;
    forEachSample([&](int16_t x, int16_t y, int16_t z) {
        const double dx = (double)x - cx;
        const double dy = (double)y - cy;
        const double dz = (double)z - cz;
        const double ri = sqrt(dx*dx + dy*dy + dz*dz);
        const double e = ri - R;
        rss += e*e;
    });
    const double rms_counts = sqrt(rss / (double)n_samples_);

    // Scale to µT for gating + reporting.
    const float R_uT = (float)R * IIS2MDC_LSB_TO_uT;
    const float res_uT = (float)rms_counts * IIS2MDC_LSB_TO_uT;

    // Clamp offsets to int16 so OFFSET_X/Y/Z can hold them.  A clamp here
    // means the underlying offset is bigger than the chip's tracking
    // range — extremely unlikely in practice (current bench reads
    // ~1640 µT ≈ 11k LSB, comfortably inside int16 ±32k).
    auto clamp_i16 = [](double v) -> int16_t {
        if (v >  32767.0) return  32767;
        if (v < -32768.0) return -32768;
        return (int16_t)lrint(v);
    };
    fit_cx_ = clamp_i16(cx);
    fit_cy_ = clamp_i16(cy);
    fit_cz_ = clamp_i16(cz);
    fit_R_uT_ = R_uT;
    fit_residual_uT_ = res_uT;
    fit_valid_ = true;

    // Recompute coverage_mask_ using samples *relative to the fit
    // centre* (issue #96 follow-up).  The original mask binned raw
    // samples by direction in mag space, which is misleading when the
    // hard-iron bias dominates: with a ~1640 µT bias every sample
    // lands in the same +X wedge even though the samples actually
    // span the offset sphere correctly.  After centring on (cx, cy, cz)
    // the samples should sit on a unit-radius shell around the origin,
    // and the wedge mask becomes a meaningful "did the user rotate
    // through the field?" indicator that drives the coverage gate
    // below and the iOS REVIEW screen's coverage row.
    {
        uint32_t post_fit_mask = 0;
        forEachSample([&](int16_t x, int16_t y, int16_t z) {
            const double dx = (double)x - cx;
            const double dy = (double)y - cy;
            const double dz = (double)z - cz;
            const double r2 = dx*dx + dy*dy + dz*dz;
            if (!(r2 > 0.0)) return;
            const double r = sqrt(r2);
            const double ux = dx / r;
            const double uy = dy / r;
            const double uz = dz / r;
            constexpr double T = 0.4;
            auto bin = [](double v) -> int {
                return (v < -T) ? 0 : (v > T) ? 2 : 1;
            };
            const uint8_t wedge = (uint8_t)(bin(ux) * 9 + bin(uy) * 3 + bin(uz));
            if (wedge < 27) post_fit_mask |= (1u << wedge);
        });
        coverage_mask_ = post_fit_mask;
    }

    // Gate the fit.  R and residual are the direct, quantitative
    // measures of fit quality: a fitted R inside the WMM band that
    // produces a sub-µT-class RMS residual is mathematically a clean
    // sphere fit through the sample cloud.  Coverage USED to be the
    // first check, but it's a poor proxy when the hard-iron bias is
    // big — samples cluster in raw mag-space regardless of rotation,
    // and even after post-fit centring the user typically only hits
    // 6-12 wedges (one per cardinal orientation), not the 18+ the
    // original threshold was set for.  Coverage stays in the status
    // frame as an informational diagnostic, but it no longer rejects
    // an otherwise-healthy fit.  Issue #96.
    if      (R_uT < MAG_CAL_R_MIN_UT)           fit_reject_code_ = MAG_CAL_REJECT_R_TOO_LOW;
    else if (R_uT > MAG_CAL_R_MAX_UT)           fit_reject_code_ = MAG_CAL_REJECT_R_TOO_HIGH;
    else if (res_uT > MAG_CAL_MAX_RESIDUAL_UT)  fit_reject_code_ = MAG_CAL_REJECT_HIGH_RESIDUAL;
    else                                         fit_reject_code_ = MAG_CAL_OK;
}
