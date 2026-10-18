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

#include <cmath>

#include "RawGnssShadow.h"
#include "RawMeasBuilder.h"
#include "ReceiverProfile.h"
#include "TR_RawGnssEKF.h"

namespace gnss_raw
{

// The period the filter's epochs come at: the fused grid's where the profile
// has one, else the receiver's own.
inline double fusedEpochPeriod(const GnssReceiverProfile& p)
{
    return p.fuse_epoch_period_s > 0.0 ? p.fuse_epoch_period_s : p.raw_epoch_period_s;
}

// reject_persist_s in epochs of period_s, at least 1 and at most what the
// EKF's per-satellite count (a uint8_t) reaches
inline int persistEpochs(double persist_s, double period_s)
{
    const long n = std::lround(persist_s / period_s);
    return n < 1 ? 1 : (n > 255 ? 255 : static_cast<int>(n));
}

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
    // the epochs it builds: consecutive ones are the fused grid's apart
    c.epoch_period_s = fusedEpochPeriod(p);
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
    // the Dopplers in pairs: the mean of two raw_epoch_period_s apart is the
    // range rate halfway between them, before the fused epoch's tag -- on the
    // epochs handed over in pairs (TimedRawEpoch::rr_paired); the rest keep
    // rr_lag_s
    e.rr_pair_lag_s = p.rr_pair_avg && dopplerPairsFit(p) ? 0.5 * p.raw_epoch_period_s : 0.0;
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
    // A refusal's let-in in time, not epochs: the Doppler counts every fused
    // epoch, the pseudorange only those it is fused on (pr_period_s)
    const double fused = fusedEpochPeriod(p);
    if (p.reject_persist_s > 0.0 && fused > 0.0)
    {
        e.reject_persist = persistEpochs(p.reject_persist_s, p.pr_period_s > fused ? p.pr_period_s : fused);
        e.reject_persist_rr = persistEpochs(p.reject_persist_s, fused);
    }
}

// The one call: the shadow's own settings, its builder's (the GNSS task's
// stage takes them through stageConfig) and its EKF's.
inline void applyReceiverProfile(const GnssReceiverProfile& p, RawGnssShadow::Config& c)
{
    c.epoch_latency_s = p.epoch_latency_s;
    c.max_epoch_age_s = p.max_epoch_age_s;
    c.epoch_job_max_s = p.epoch_job_max_s;
    c.pr_period_s = p.pr_period_s;
    c.rr_dyn_window_s = p.rr_dyn_window_s;
    c.heading_fit_period_s = p.heading_fit_period_s;
    c.sat_cap = p.sat_cap;
    c.level.min_dt_s = p.level_min_dt_s;
    applyReceiverProfile(p, c.meas);
    applyReceiverProfile(p, c.ekf);
}

// The mosaic-only image's flight safeguards (wave 3, 2026-10-18), in one place:
// main.cpp's startRawShadow calls it under TR_RAW_NAV_MOSAIC after the profile
// and before any bench override, and tools/raw_replay's --receiver mosaic calls
// it too, so a replay flies the image's settings by construction. The LC86G
// and generic paths never call it: every field below defaults off, the
// Beetle's filter. Each one fails toward unaided -- the check holds Dopplers
// out, the share lowers GNSS's heading gain, the allowance widens the
// Dopplers' sigma -- and none needs the antenna's offset, which is never
// measured before a flight (a bound only).
//   - The Doppler change check (RawGnssShadow.h), acting in flight only: each
//     Doppler's change between epochs against the IMU's velocity change along
//     its line of sight, summed over a leaky 1 s window, the epoch's change
//     at 4 sigmas with a 10 deg attitude allowance, and a failed epoch's
//     latch let go at 5. Grid B: a loop settling after the thrust, a stale
//     receiver and the slow ramps no longer turn the coast's heading (worst
//     pre-apogee excess over the unaided twin 62 -> 1.3 deg on the settling
//     pattern), with no-fault runs unchanged run for run. A launch with no
//     magnetometer heading (the V9's until its magnetometer is calibrated in
//     flight configuration) takes any heading about the vertical as well
//     (dv_hdg_unset_deg 180): with a pad heading 120-180 deg off, the 10 deg
//     allowance alone held the Dopplers out of no-fault flights.
//   - GNSS's heading gain by how well the moment observes it (TR_RawGnssEKF.h
//     hdg_obs_mode): the rows' gain on the attitude about the vertical and the
//     gyro bias along it scaled by the 1 s mean horizontal specific force over
//     5 m/s^2, open whenever the IMU read past 2 g within 0.2 s, everywhere
//     (the pad too). It ends the real logs' heading walk on a slowly turning
//     pad and halves the consistent ramps the check cannot see.
//   - The turning body's Doppler allowance: each Doppler's sigma takes what
//     an antenna up to 0.3 m along the nose and 0.03 m across it could move
//     while the body turns, on an epoch whose rate across the nose reached
//     2 rad/s (the drogue's snatch, not a boost's weathercocking), at any time
//     (motor ejection can snatch before the apogee call). The hold and the
//     skip stay off.
inline void applyMosaicFlightSafeguards(RawGnssShadow::Config& c)
{
    // the Doppler change check
    c.dv_check = RawGnssShadow::kDvAct;
    c.dv_window_s = 1.0;
    c.dv_att_allow_deg = 10.0;
    c.dv_epoch_k = 4.0;
    c.dv_latch_k = 5.0;
    c.dv_ground = false;
    c.dv_sat_k = 5.0;
    c.dv_floor_mps = 0.03;
    c.dv_sigma_per_g = 0.1;
    c.dv_scale_allow = 0.01;
    c.dv_bias_allow_mps2 = 0.1;
    c.dv_lever_allow_m = 0.3;   // a bound on the offset, not a measured one
    c.dv_hold_s = 1.0;
    c.dv_epoch_hold_s = 0.5;
    c.dv_deweight = false;
    c.dv_noise_tau_s = 0.0;
    c.dv_hold_all_frac = 0.5;
    c.dv_hdg_unset_deg = 180.0;   // no magnetometer heading at launch: any heading
    // GNSS's heading gain by its observability
    c.ekf.hdg_obs_mode = kHdgObsForce;
    c.ekf.hdg_obs_fh_mps2 = 5.0;
    c.ekf.hdg_obs_tau_s = 1.0;
    c.ekf.hdg_obs_open_g = 2.0;
    c.ekf.hdg_obs_open_hold_s = 0.2;
    c.ekf.hdg_obs_floor = 0.0;
    c.heading_obs_scope = RawGnssShadow::kObsEverywhere;
    // the turning body's Doppler allowance (bounds on the antenna's offset,
    // not measured ones); its attitude hold and its epoch skip stay off
    c.rot_lever_axial_m = 0.30;
    c.rot_lever_radial_m = 0.03;
    c.rot_lever_min_rps = 2.0;
    c.rot_after_apogee = false;
    c.att_hold_rate_rps = 0.0;
    c.att_hold_s = 0.0;
    c.rot_skip_rate_rps = 0.0;
    c.rot_skip_s = 0.0;
    // where the owner measured the antenna's offset (applyAntennaOffset, below):
    // what it could be off by, in place of the bounds above (Grid B, 0.10 m up
    // the nose: 3 to 10 cm all take the snatch's worst excess from 13 deg to
    // 3; 5 and 2 cm keep an offset given 5 cm short the closest to it)
    c.antenna_sigma_axial_m = 0.05;
    c.antenna_sigma_radial_m = 0.02;
    c.antenna_min_rps = 0.0;
    c.antenna_att_mode = kLeverAttObserve;
}

// The antenna's offset from the IMU, where the rocket's owner measured it: an
// optional per-rocket setting (the apps' Antenna offset, BLE cmd 81), unknown
// unless set.
struct AntennaOffset
{
    bool   known = false;
    double b_m[3] = {0.0, 0.0, 0.0};   // body FRD (the filter's frame), the IMU to the antenna's phase centre, m
};

// The owner's setting (BLE cmd 81, AntennaOffsetConfigData: the board's own
// frame, FLU, mm) in the filter's: turned into the rocket's frame by the
// mounting the flight computer turns the IMU by (b2r: row-major, v_rocket = R
// v_board, both FLU), then FRD (y and z negated), in metres. Unknown, or a
// setting that is no setting (antennaOffsetValid), is unknown.
inline AntennaOffset antennaOffsetInBody(const AntennaOffsetConfigData& s, const float b2r[9])
{
    AntennaOffset a;
    if (s.known != 1 || !antennaOffsetValid(s))
        return a;
    const double rb[3] = {s.x_mm * 1.0e-3, s.y_mm * 1.0e-3, s.z_mm * 1.0e-3};
    double rr[3];
    for (int i = 0; i < 3; ++i)
        rr[i] = static_cast<double>(b2r[3 * i]) * rb[0] + static_cast<double>(b2r[3 * i + 1]) * rb[1] +
                static_cast<double>(b2r[3 * i + 2]) * rb[2];
    a.known = true;
    a.b_m[0] = rr[0];
    a.b_m[1] = -rr[1];
    a.b_m[2] = -rr[2];
    return a;
}

// The offset on the filter (RawGnssShadow.h, the antenna's offset): known,
// every Doppler and pseudorange is predicted at the antenna, and what the
// offset could be off by (antenna_sigma_*, applyMosaicFlightSafeguards)
// takes the place of the bounds on the Dopplers and in the change check.
// Unknown changes nothing: the filter as the image flies it without one.
// After the safeguards; RawGnssShadow::setAntenna does the same on the ground.
inline void applyAntennaOffset(const AntennaOffset& a, RawGnssShadow::Config& c)
{
    if (!a.known)
        return;
    c.ekf.lever_on = true;
    for (int i = 0; i < 3; ++i)
        c.ekf.lever_b_m[i] = a.b_m[i];
    c.ekf.lever_att_mode = c.antenna_att_mode;
}

}  // namespace gnss_raw

#endif  // RAW_GNSS_RECEIVER_PROFILE_APPLY_H
