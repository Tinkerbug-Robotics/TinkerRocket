// ReceiverProfileApply.h — a receiver's profile (TR_GNSSRaw/ReceiverProfile.h)
// into the raw-GNSS filter's configs (#1600): the measurement builder's
// (RawMeasBuilder::Config), the EKF's (RawGnssEKFParams) and the shadow's,
// which holds both (RawGnssShadow::Config, and through it the GNSS task's
// stage: RawGnssShadow::stageConfig). The flight computer picks the board's
// receiver's profile and applies it before any bench override (main.cpp's
// startRawShadow); the sim's bindings apply one to a builder or a filter.
//
// Header-only: built where it is asked for (on the flight computer once, at
// start, in flash).
#ifndef RAW_GNSS_RECEIVER_PROFILE_APPLY_H
#define RAW_GNSS_RECEIVER_PROFILE_APPLY_H

#include "RawGnssShadow.h"
#include "RawMeasBuilder.h"
#include "ReceiverProfile.h"
#include "TR_RawGnssEKF.h"

namespace gnss_raw
{

inline void applyReceiverProfile(const GnssReceiverProfile& p, RawMeasBuilder::Config& c)
{
    for (int k = 0; k < kProfileSys; ++k)
        c.sig[k] = p.sig[k];
    c.pr_white_a = p.pr_white_a;
    c.pr_white_b = p.pr_white_b;
    c.pr_corr_a = p.pr_corr_a;
    c.pr_corr_b = p.pr_corr_b;
    c.pr_tau_s = p.pr_tau_s;
    c.rr_white_a = p.rr_white_a;
    c.rr_white_b = p.rr_white_b;
    c.rr_tau_s = p.rr_tau_s;
    c.doppler_step_hz = p.doppler_step_hz;
    c.pr_corr_scale = p.pr_corr_scale;
    c.rr_scale = p.rr_scale;
    c.relock_sigma_m = p.relock_sigma_m;
    c.relock_tau_s = p.relock_tau_s;
    c.relock_gap_s = p.relock_gap_s;
    c.rr_dyn_sigma = p.rr_dyn_sigma;
    c.rr_relock_sigma = p.rr_relock_sigma;
    c.carrier = p.cr_mode != kCarrierOff;
    c.cr_min_lock_ms = p.cr_min_lock_ms;
    c.epoch_period_s = p.raw_epoch_period_s;
    c.cr_max_interval_s = p.cr_max_interval_s;
    c.cr_sigma_floor_mps = p.cr_sigma_floor_mps;
}

inline void applyReceiverProfile(const GnssReceiverProfile& p, RawGnssEKFParams& e)
{
    e.clk_bias_psd = p.clk_bias_psd;
    e.clk_drift_psd = p.clk_drift_psd;
    e.isb_psd = p.isb_psd;
    e.clock_step_m = p.clock_step_m;
    e.clock_step_spread_m = p.clock_step_spread_m;
    e.drift_step_mps = p.drift_step_mps;
    e.drift_step_spread_mps = p.drift_step_spread_mps;
    e.p0_clk_bias = p.p0_clk_bias;
    e.p0_clk_drift = p.p0_clk_drift;
    e.p0_isb = p.p0_isb;
    e.rr_lag_s = p.rr_lag_s;
    e.cr_mode = p.cr_mode;
    e.cr_sigma_per_rev_s = p.cr_sigma_per_rev_s;
    e.cr_common_sigma_mps = p.cr_common_sigma_mps;
    e.cr_screen_k = p.cr_screen_k;
    e.cr_screen_floor_mps = p.cr_screen_floor_mps;
    e.cr_half_cycle = p.cr_half_cycle;
    for (int k = 0; k < kProfileSys; ++k)
    {
        e.cr_carrier_hz[k] = p.sig[k].carrier_hz;
        e.cr_half_cycle_unflagged[k] = p.sig[k].half_cycle_unflagged;
    }
}

// The one call: the shadow's own settings, its builder's (the GNSS task's
// stage takes them through stageConfig) and its EKF's.
inline void applyReceiverProfile(const GnssReceiverProfile& p, RawGnssShadow::Config& c)
{
    c.epoch_latency_s = p.epoch_latency_s;
    applyReceiverProfile(p, c.meas);
    applyReceiverProfile(p, c.ekf);
}

}  // namespace gnss_raw

#endif  // RAW_GNSS_RECEIVER_PROFILE_APPLY_H
