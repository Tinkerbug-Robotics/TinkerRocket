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
      state_(State::IDLE)
{}

void MagCalibrator::start()
{
    n_samples_ = 0;
    coverage_mask_ = 0;
    last_x_ = last_y_ = last_z_ = 0;
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
    state_ = State::APPLIED;
#ifdef ESP_PLATFORM
    ESP_LOGI(TAG, "accept: APPLIED (cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT)",
             (int)fit_cx_, (int)fit_cy_, (int)fit_cz_,
             (double)fit_R_uT_, (double)fit_residual_uT_);
#endif
    return true;
}

void MagCalibrator::clear()
{
    state_ = State::IDLE;
}

bool MagCalibrator::addSample(int16_t x, int16_t y, int16_t z)
{
    if (state_ != State::SAMPLING) return false;
    if (n_samples_ >= MAX_SAMPLES) return false;

    samples_x_[n_samples_] = x;
    samples_y_[n_samples_] = y;
    samples_z_[n_samples_] = z;
    n_samples_++;

    last_x_ = x;
    last_y_ = y;
    last_z_ = z;

    const uint8_t wedge = directionWedge(x, y, z);
    if (wedge < 27) coverage_mask_ |= (1u << wedge);

    if (n_samples_ >= MAX_SAMPLES)
    {
        runFit();
        state_ = State::REVIEW;
#ifdef ESP_PLATFORM
        ESP_LOGI(TAG, "fit complete: cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT cov=%u/26 reject=%u",
                 (int)fit_cx_, (int)fit_cy_, (int)fit_cz_,
                 (double)fit_R_uT_, (double)fit_residual_uT_,
                 (unsigned)__builtin_popcount(coverage_mask_),
                 (unsigned)fit_reject_code_);
#endif
        return true;
    }
    return false;
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
        case State::IDLE:     out.sub_type = MAG_CAL_SUB_IDLE;     break;
        case State::SAMPLING: out.sub_type = MAG_CAL_SUB_SAMPLING; break;
        case State::REVIEW:   out.sub_type = MAG_CAL_SUB_REVIEW;   break;
        case State::APPLIED:  out.sub_type = MAG_CAL_SUB_APPLIED;  break;
        case State::ABORTED:  out.sub_type = MAG_CAL_SUB_ABORTED;  break;
    }

    out.coverage_bins = (uint8_t)__builtin_popcount(coverage_mask_);
    out.coverage_mask = coverage_mask_;
    out.sample_count  = n_samples_;

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

uint8_t MagCalibrator::directionWedge(int16_t x, int16_t y, int16_t z)
{
    // Reject the zero vector (sample exactly at origin — implausible from
    // a real sensor, but defends against a degenerate path).
    const double xx = (double)x * (double)x;
    const double yy = (double)y * (double)y;
    const double zz = (double)z * (double)z;
    const double r2 = xx + yy + zz;
    if (r2 <= 0.0) return 27;  // sentinel = invalid

    const double r = sqrt(r2);
    const double ux = (double)x / r;
    const double uy = (double)y / r;
    const double uz = (double)z / r;

    // Threshold = 0.4: a unit vector with all three |components| < 0.4
    // would need r < sqrt(3) * 0.4 ≈ 0.69, which is impossible — so the
    // (0,0,0) center cell never lights up and we get exactly 26 reachable
    // wedges.
    constexpr double T = 0.4;
    auto bin = [](double v) -> int {
        return (v < -T) ? 0 : (v > T) ? 2 : 1;
    };
    const int bx = bin(ux);
    const int by = bin(uy);
    const int bz = bin(uz);
    return (uint8_t)(bx * 9 + by * 3 + bz);  // 0..26 (13 = center, never set)
}

void MagCalibrator::runFit()
{
    fit_valid_ = false;
    fit_cx_ = fit_cy_ = fit_cz_ = 0;
    fit_R_uT_ = 0.0f;
    fit_residual_uT_ = 0.0f;
    fit_reject_code_ = MAG_CAL_OK;

    if (n_samples_ < MAG_CAL_MIN_SAMPLES) return;

    // Accumulate sums in double.  Centroid-shifting before solving keeps
    // the cross-product sums small (cancel the bulk of the offset before
    // squaring), which preserves ~7+ digits of precision in the normal
    // equations even with raw counts in the ~1e4 range.
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (uint16_t i = 0; i < n_samples_; i++)
    {
        sx += (double)samples_x_[i];
        sy += (double)samples_y_[i];
        sz += (double)samples_z_[i];
    }
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

    for (uint16_t i = 0; i < n_samples_; i++)
    {
        const double xc = (double)samples_x_[i] - mx;
        const double yc = (double)samples_y_[i] - my;
        const double zc = (double)samples_z_[i] - mz;
        const double bi = xc*xc + yc*yc + zc*zc;

        sxx += xc*xc; syy += yc*yc; szz += zc*zc;
        sxy += xc*yc; sxz += xc*zc; syz += yc*zc;
        sx_c += xc; sy_c += yc; sz_c += zc;
        sb += bi;
        sxb += xc * bi;
        syb += yc * bi;
        szb += zc * bi;
    }

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
    for (uint16_t i = 0; i < n_samples_; i++)
    {
        const double dx = (double)samples_x_[i] - cx;
        const double dy = (double)samples_y_[i] - cy;
        const double dz = (double)samples_z_[i] - cz;
        const double ri = sqrt(dx*dx + dy*dy + dz*dz);
        const double e = ri - R;
        rss += e*e;
    }
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

    // Gate the fit.  Order: coverage first (fastest reason to retry),
    // then R band, then residual.  These are advisory codes — caller
    // shows the right error in the iOS UI.
    const uint8_t cov = (uint8_t)__builtin_popcount(coverage_mask_);
    if (cov < MAG_CAL_MIN_COVERAGE_BINS)        fit_reject_code_ = MAG_CAL_REJECT_LOW_COVERAGE;
    else if (R_uT < MAG_CAL_R_MIN_UT)           fit_reject_code_ = MAG_CAL_REJECT_R_TOO_LOW;
    else if (R_uT > MAG_CAL_R_MAX_UT)           fit_reject_code_ = MAG_CAL_REJECT_R_TOO_HIGH;
    else if (res_uT > MAG_CAL_MAX_RESIDUAL_UT)  fit_reject_code_ = MAG_CAL_REJECT_HIGH_RESIDUAL;
    else                                         fit_reject_code_ = MAG_CAL_OK;
}
