// Host tests for the receiver profiles (#1600): TR_GNSSRaw/ReceiverProfile.h
// and TR_RawGnssShadow/ReceiverProfileApply.h. The LC86G's profile, applied,
// is exactly what the builder and the filter flew with before there were
// profiles -- and the configs' own defaults, so a default-constructed builder
// or filter is the LC86G's; the stubs are its numbers, marked UNMEASURED; the
// mosaic-G5's v0, every number pinned; the carrier phase's sigma between its
// C/N0 anchors. And the Beetle's tripwires: every config's size (a new field
// must join the comparators), every RawGnssShadow::Config default pinned, the
// raw epoch's and the ephemeris store's capacities.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <type_traits>

#include "EphemerisStore.h"
#include "RawGnssShadow.h"
#include "RawMeas.h"
#include "RawMeasBuilder.h"
#include "ReceiverProfile.h"
#include "ReceiverProfileApply.h"
#include "TR_RawGnssEKF.h"
#include "gnss/types.h"

using gnss_raw::GnssReceiverProfile;
using gnss_raw::GnssSignalProfile;
using gnss_raw::RawGnssEKFParams;
using gnss_raw::RawGnssShadow;
using gnss_raw::RawMeasBuilder;

namespace
{

void expectSameSignal(const GnssSignalProfile& a, const GnssSignalProfile& b)
{
    EXPECT_EQ(a.carrier_hz, b.carrier_hz);
    EXPECT_EQ(a.cn0_min_pr, b.cn0_min_pr);
    EXPECT_EQ(a.cn0_min_rr, b.cn0_min_rr);
    EXPECT_EQ(a.cn0_min_cr, b.cn0_min_cr);
    EXPECT_EQ(a.phase_sigma35_m, b.phase_sigma35_m);
    EXPECT_EQ(a.phase_sigma45_m, b.phase_sigma45_m);
    EXPECT_EQ(a.half_cycle_unflagged, b.half_cycle_unflagged);
}

void expectSameBuilder(const RawMeasBuilder::Config& a, const RawMeasBuilder::Config& b)
{
    EXPECT_EQ(a.el_mask_deg, b.el_mask_deg);
    EXPECT_EQ(a.troposphere, b.troposphere);
    for (int k = 0; k < gnss_raw::kProfileSys; ++k)
    {
        SCOPED_TRACE(testing::Message() << "sys " << k);
        expectSameSignal(a.sig[k], b.sig[k]);
    }
    EXPECT_EQ(a.pr_white_a, b.pr_white_a);
    EXPECT_EQ(a.pr_white_b, b.pr_white_b);
    EXPECT_EQ(a.pr_corr_a, b.pr_corr_a);
    EXPECT_EQ(a.pr_corr_b, b.pr_corr_b);
    EXPECT_EQ(a.pr_tau_s, b.pr_tau_s);
    EXPECT_EQ(a.rr_white_a, b.rr_white_a);
    EXPECT_EQ(a.rr_white_b, b.rr_white_b);
    EXPECT_EQ(a.rr_tau_s, b.rr_tau_s);
    EXPECT_EQ(a.doppler_step_hz, b.doppler_step_hz);
    EXPECT_EQ(a.pr_corr_scale, b.pr_corr_scale);
    EXPECT_EQ(a.rr_scale, b.rr_scale);
    EXPECT_EQ(a.relock_sigma_m, b.relock_sigma_m);
    EXPECT_EQ(a.relock_tau_s, b.relock_tau_s);
    EXPECT_EQ(a.relock_gap_s, b.relock_gap_s);
    EXPECT_EQ(a.rr_dyn_sigma, b.rr_dyn_sigma);
    EXPECT_EQ(a.rr_relock_sigma, b.rr_relock_sigma);
    EXPECT_EQ(a.sat_refresh_s, b.sat_refresh_s);
    EXPECT_EQ(a.carrier, b.carrier);
    EXPECT_EQ(a.cr_min_lock_ms, b.cr_min_lock_ms);
    EXPECT_EQ(a.epoch_period_s, b.epoch_period_s);
    EXPECT_EQ(a.cr_max_interval_s, b.cr_max_interval_s);
    EXPECT_EQ(a.cr_sigma_floor_mps, b.cr_sigma_floor_mps);
}

void expectSameEkf(const RawGnssEKFParams& a, const RawGnssEKFParams& b)
{
    EXPECT_EQ(a.a_noise, b.a_noise);
    EXPECT_EQ(a.a_markov_sigma, b.a_markov_sigma);
    EXPECT_EQ(a.a_markov_tau, b.a_markov_tau);
    EXPECT_EQ(a.w_noise, b.w_noise);
    EXPECT_EQ(a.w_markov_sigma, b.w_markov_sigma);
    EXPECT_EQ(a.w_markov_tau, b.w_markov_tau);
    EXPECT_EQ(a.accel_var, b.accel_var);
    EXPECT_EQ(a.level_heading_free, b.level_heading_free);
    EXPECT_EQ(a.mag_sigma_ut, b.mag_sigma_ut);
    EXPECT_EQ(a.mag_tilt_sigma_mps2, b.mag_tilt_sigma_mps2);
    EXPECT_EQ(a.mag_r_floor, b.mag_r_floor);
    EXPECT_EQ(a.mag_r_ceiling, b.mag_r_ceiling);
    EXPECT_EQ(a.mag_bias_nis, b.mag_bias_nis);
    EXPECT_EQ(a.clk_bias_psd, b.clk_bias_psd);
    EXPECT_EQ(a.clk_drift_psd, b.clk_drift_psd);
    EXPECT_EQ(a.isb_psd, b.isb_psd);
    EXPECT_EQ(a.gate_chi2, b.gate_chi2);
    EXPECT_EQ(a.reject_persist, b.reject_persist);
    EXPECT_EQ(a.reject_persist_rr, b.reject_persist_rr);
    EXPECT_EQ(a.clock_step_m, b.clock_step_m);
    EXPECT_EQ(a.clock_step_spread_m, b.clock_step_spread_m);
    EXPECT_EQ(a.drift_step_mps, b.drift_step_mps);
    EXPECT_EQ(a.drift_step_spread_mps, b.drift_step_spread_mps);
    EXPECT_EQ(a.rr_lag_s, b.rr_lag_s);
    EXPECT_EQ(a.att_gate_cos4, b.att_gate_cos4);
    EXPECT_EQ(a.att_gate_vertical, b.att_gate_vertical);
    EXPECT_EQ(a.att_gnss_off, b.att_gnss_off);
    EXPECT_EQ(a.hdg_obs_mode, b.hdg_obs_mode);
    EXPECT_EQ(a.hdg_obs_fh_mps2, b.hdg_obs_fh_mps2);
    EXPECT_EQ(a.hdg_obs_tau_s, b.hdg_obs_tau_s);
    EXPECT_EQ(a.hdg_obs_sigma_v, b.hdg_obs_sigma_v);
    EXPECT_EQ(a.hdg_obs_window_s, b.hdg_obs_window_s);
    EXPECT_EQ(a.hdg_obs_open_g, b.hdg_obs_open_g);
    EXPECT_EQ(a.hdg_obs_open_hold_s, b.hdg_obs_open_hold_s);
    EXPECT_EQ(a.hdg_obs_floor, b.hdg_obs_floor);
    EXPECT_EQ(a.p0_pos, b.p0_pos);
    EXPECT_EQ(a.p0_vel, b.p0_vel);
    EXPECT_EQ(a.p0_att, b.p0_att);
    EXPECT_EQ(a.p0_hdg, b.p0_hdg);
    EXPECT_EQ(a.p0_abias, b.p0_abias);
    EXPECT_EQ(a.p0_wbias, b.p0_wbias);
    EXPECT_EQ(a.p0_clk_bias, b.p0_clk_bias);
    EXPECT_EQ(a.p0_clk_drift, b.p0_clk_drift);
    EXPECT_EQ(a.p0_isb, b.p0_isb);
    EXPECT_EQ(a.gravity_wgs84, b.gravity_wgs84);
    EXPECT_EQ(a.hist_spacing_s, b.hist_spacing_s);
    EXPECT_EQ(a.cr_mode, b.cr_mode);
    EXPECT_EQ(a.cr_sigma_per_rev_s, b.cr_sigma_per_rev_s);
    EXPECT_EQ(a.cr_common_sigma_mps, b.cr_common_sigma_mps);
    EXPECT_EQ(a.cr_screen_k, b.cr_screen_k);
    EXPECT_EQ(a.cr_screen_floor_mps, b.cr_screen_floor_mps);
    EXPECT_EQ(a.cr_half_cycle, b.cr_half_cycle);
    for (int k = 0; k < gnss_raw::kProfileSys; ++k)
    {
        EXPECT_EQ(a.cr_carrier_hz[k], b.cr_carrier_hz[k]) << "sys " << k;
        EXPECT_EQ(a.cr_half_cycle_unflagged[k], b.cr_half_cycle_unflagged[k]) << "sys " << k;
    }
}

// The shadow's own settings a profile sets (the rest are its builder's and
// its EKF's, above).
void expectSameShadowSettings(const RawGnssShadow::Config& a, const RawGnssShadow::Config& b)
{
    EXPECT_EQ(a.epoch_latency_s, b.epoch_latency_s);
    EXPECT_EQ(a.max_epoch_age_s, b.max_epoch_age_s);
    EXPECT_EQ(a.epoch_job_max_s, b.epoch_job_max_s);
    EXPECT_EQ(a.pr_period_s, b.pr_period_s);
    EXPECT_EQ(a.rr_dyn_window_s, b.rr_dyn_window_s);
    EXPECT_EQ(a.heading_fit_period_s, b.heading_fit_period_s);
    EXPECT_EQ(a.sat_cap, b.sat_cap);
    EXPECT_EQ(a.level.min_dt_s, b.level.min_dt_s);
}

// Every field of a profile but its name, type and measured mark.
void expectSameNumbers(const GnssReceiverProfile& a, const GnssReceiverProfile& b)
{
    for (int k = 0; k < gnss_raw::kProfileSys; ++k)
        expectSameSignal(a.sig[k], b.sig[k]);
    RawMeasBuilder::Config ca, cb;
    gnss_raw::applyReceiverProfile(a, ca);
    gnss_raw::applyReceiverProfile(b, cb);
    expectSameBuilder(ca, cb);
    RawGnssEKFParams ea, eb;
    gnss_raw::applyReceiverProfile(a, ea);
    gnss_raw::applyReceiverProfile(b, eb);
    expectSameEkf(ea, eb);
    EXPECT_EQ(a.epoch_latency_s, b.epoch_latency_s);
    EXPECT_EQ(a.raw_epoch_period_s, b.raw_epoch_period_s);
    EXPECT_EQ(a.cr_mode, b.cr_mode);
    EXPECT_EQ(a.cr_half_cycle, b.cr_half_cycle);
    EXPECT_EQ(a.fuse_epoch_period_s, b.fuse_epoch_period_s);
    EXPECT_EQ(a.pr_period_s, b.pr_period_s);
    EXPECT_EQ(a.sat_cap, b.sat_cap);
    EXPECT_EQ(a.reject_persist_s, b.reject_persist_s);
    EXPECT_EQ(a.level_min_dt_s, b.level_min_dt_s);
    EXPECT_EQ(a.heading_fit_period_s, b.heading_fit_period_s);
    EXPECT_EQ(a.rr_dyn_window_s, b.rr_dyn_window_s);
    EXPECT_EQ(a.max_epoch_age_s, b.max_epoch_age_s);
    EXPECT_EQ(a.epoch_job_max_s, b.epoch_job_max_s);
    RawGnssShadow::Config sa, sb;
    gnss_raw::applyReceiverProfile(a, sa);
    gnss_raw::applyReceiverProfile(b, sb);
    expectSameShadowSettings(sa, sb);
}

// The Beetle's tripwires. A field added to any of these configs must join
// the comparators above (and, for the shadow's, the pins in
// TheShadowSafeguardDefaultsAreTheBeetles) before its size goes in here: the
// Beetle's filter is its defaults, and the M1 build applies the LC86G's
// profile over them. The sizes are a 64-bit host's (macOS and Linux alike).
static_assert(sizeof(void*) != 8 || sizeof(RawGnssEKFParams) == 424,
              "RawGnssEKFParams changed: add the field to expectSameEkf, then update this size");
static_assert(sizeof(void*) != 8 || sizeof(RawMeasBuilder::Config) == 312,
              "RawMeasBuilder::Config changed: add the field to expectSameBuilder, then update this size");
static_assert(sizeof(void*) != 8 || sizeof(GnssSignalProfile) == 32,
              "GnssSignalProfile changed: add the field to expectSameSignal, then update this size");
static_assert(sizeof(void*) != 8 || sizeof(GnssReceiverProfile) == 520,
              "GnssReceiverProfile changed: add the field to expectSameNumbers, then update this size");
static_assert(sizeof(void*) != 8 || sizeof(RawGnssShadow::Config) == 1600,
              "RawGnssShadow::Config changed: pin the field in TheShadowSafeguardDefaultsAreTheBeetles, then update "
              "this size");

}  // namespace

TEST(ReceiverProfile, TheLc86gProfileIsWhatTheFilterFlewWith)
{
    // The values every builder and filter took before profiles (a921aa41),
    // literally: the profile applied must give them
    RawMeasBuilder::Config b;
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), b);
    EXPECT_EQ(b.el_mask_deg, 3.0);
    EXPECT_TRUE(b.troposphere);
    EXPECT_EQ(b.doppler_step_hz, 0.0);
    EXPECT_EQ(b.pr_corr_scale, 0.5);
    EXPECT_EQ(b.rr_scale, 1.0);
    EXPECT_EQ(b.relock_sigma_m, 40.0);
    EXPECT_EQ(b.relock_tau_s, 10.0);
    EXPECT_EQ(b.relock_gap_s, 2.5);
    EXPECT_EQ(b.rr_dyn_sigma, 0.5);
    EXPECT_EQ(b.rr_relock_sigma, 3.0);
    EXPECT_EQ(b.sat_refresh_s, 5.0);
    // the noise model's fit, the builder's constants
    EXPECT_EQ(b.pr_white_a, 0.026);
    EXPECT_EQ(b.pr_white_b, 0.157);
    EXPECT_EQ(b.pr_corr_a, 1.94);
    EXPECT_EQ(b.pr_corr_b, 3.5);
    EXPECT_EQ(b.pr_tau_s, 15.0);
    EXPECT_EQ(b.rr_white_a, 0.0);
    EXPECT_EQ(b.rr_white_b, 0.2);
    EXPECT_EQ(b.rr_tau_s, 0.1);
    EXPECT_EQ(b.pr_white_a, RawMeasBuilder::kPrWhiteA);
    EXPECT_EQ(b.rr_white_b, RawMeasBuilder::kRrWhiteB);
    // the carriers it scaled the ionosphere and the Doppler's step by: L1 C/A, E1, B1I
    EXPECT_EQ(b.sig[GNSS_SYS_GPS].carrier_hz, GNSS_FREQ_L1_HZ);
    EXPECT_EQ(b.sig[GNSS_SYS_GAL].carrier_hz, GNSS_FREQ_L1_HZ);
    EXPECT_EQ(b.sig[GNSS_SYS_BDS].carrier_hz, 1561.098e6);
    // no C/N0 floor on the code or the Doppler
    for (int k = 0; k < gnss_raw::kProfileSys; ++k)
    {
        EXPECT_EQ(b.sig[k].cn0_min_pr, 0.0f);
        EXPECT_EQ(b.sig[k].cn0_min_rr, 0.0f);
    }

    RawGnssEKFParams e;
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), e);
    EXPECT_EQ(e.rr_lag_s, 0.0555);
    EXPECT_EQ(e.clk_bias_psd, 0.009);
    EXPECT_EQ(e.clk_drift_psd, 100.0);
    EXPECT_EQ(e.isb_psd, 1.0e-4);
    EXPECT_EQ(e.clock_step_m, 1000.0);
    EXPECT_EQ(e.clock_step_spread_m, 200.0);
    EXPECT_EQ(e.drift_step_mps, 10.0);
    EXPECT_EQ(e.drift_step_spread_mps, 2.0);
    EXPECT_EQ(e.p0_clk_bias, 30.0);
    EXPECT_EQ(e.p0_clk_drift, 5.0);
    EXPECT_EQ(e.p0_isb, 30.0);
    EXPECT_EQ(e.gate_chi2, 10.83);
    EXPECT_EQ(e.reject_persist, 3);

    RawGnssShadow::Config s;
    s.epoch_latency_s = 0.25;                 // anything: the profile's is the old default, 0
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), s);
    EXPECT_EQ(s.epoch_latency_s, 0.0);
    EXPECT_EQ(s.meas.relock_sigma_m, 40.0);
    EXPECT_EQ(s.ekf.rr_lag_s, 0.0555);
}

TEST(ReceiverProfile, TheConfigsDefaultsAreTheLc86gProfile)
{
    // Applied to a default config, the LC86G's profile changes nothing: the
    // defaults are its values, the new ones (the carrier's) included
    const GnssReceiverProfile p = gnss_raw::lc86gProfile();
    RawMeasBuilder::Config b0, b1;
    gnss_raw::applyReceiverProfile(p, b1);
    expectSameBuilder(b1, b0);
    RawGnssEKFParams e0, e1;
    gnss_raw::applyReceiverProfile(p, e1);
    expectSameEkf(e1, e0);
    RawGnssShadow::Config s0, s1;
    gnss_raw::applyReceiverProfile(p, s1);
    EXPECT_EQ(s1.epoch_latency_s, s0.epoch_latency_s);
    expectSameBuilder(s1.meas, s0.meas);
    expectSameEkf(s1.ekf, s0.ekf);
    // and the shadow's own: the 10 Hz switches off, its epoch ages its own
    expectSameShadowSettings(s1, s0);
    // and the GNSS task's stage takes the builder's through the shadow's config
    expectSameBuilder(RawGnssShadow::stageConfig(s1).meas, b0);
}

TEST(ReceiverProfile, TheLc86gCarrierSettings)
{
    const GnssReceiverProfile p = gnss_raw::lc86gProfile();
    EXPECT_STREQ(p.name, "LC86G");
    EXPECT_EQ(p.type, gnss_raw::kRxLc86g);
    EXPECT_TRUE(p.measured);
    EXPECT_EQ(p.cr_mode, gnss_raw::kCarrierObserve);      // observed, never fused, by default
    EXPECT_EQ(p.cr_half_cycle, gnss_raw::kHalfCycleReject);
    EXPECT_EQ(p.raw_epoch_period_s, 1.0);
    EXPECT_EQ(p.cr_min_lock_ms, 1000u);
    EXPECT_EQ(p.cr_max_interval_s, 1.5);
    EXPECT_EQ(p.cr_sigma_floor_mps, 0.008);
    EXPECT_EQ(p.cr_sigma_per_rev_s, 0.023);
    EXPECT_EQ(p.cr_common_sigma_mps, 5.0);
    EXPECT_EQ(p.cr_screen_k, 4.0);
    EXPECT_EQ(p.cr_screen_floor_mps, 0.03);
    const int sys[4] = {GNSS_SYS_GPS, GNSS_SYS_GAL, GNSS_SYS_BDS, GNSS_SYS_QZS};
    const float floor_cr[4] = {32.5f, 35.0f, 32.5f, 32.5f};
    const float s35[4] = {0.0045f, 0.0024f, 0.0045f, 0.0045f};
    const float s45[4] = {0.0014f, 0.0011f, 0.0014f, 0.0014f};
    const bool unflagged[4] = {true, false, true, true};
    for (int i = 0; i < 4; ++i)
    {
        const GnssSignalProfile& g = p.sig[sys[i]];
        EXPECT_EQ(g.cn0_min_cr, floor_cr[i]) << "sys " << sys[i];
        EXPECT_EQ(g.phase_sigma35_m, s35[i]) << "sys " << sys[i];
        EXPECT_EQ(g.phase_sigma45_m, s45[i]) << "sys " << sys[i];
        EXPECT_EQ(g.half_cycle_unflagged, unflagged[i]) << "sys " << sys[i];
    }
    // off: nothing built
    GnssReceiverProfile off = p;
    off.cr_mode = gnss_raw::kCarrierOff;
    RawMeasBuilder::Config b;
    gnss_raw::applyReceiverProfile(off, b);
    EXPECT_FALSE(b.carrier);
    RawGnssEKFParams e;
    gnss_raw::applyReceiverProfile(off, e);
    EXPECT_EQ(e.cr_mode, gnss_raw::kCarrierOff);
}

TEST(ReceiverProfile, TheStubsAreTheLc86gsNumbersMarkedUnmeasured)
{
    const GnssReceiverProfile lc = gnss_raw::lc86gProfile();
    struct
    {
        GnssReceiverProfile p;
        gnss_raw::GnssReceiverType type;
        const char* name;
    } stubs[] = {{gnss_raw::neoM8tProfile(), gnss_raw::kRxNeoM8t, "NEO-M8T"},
                 {gnss_raw::px1105rProfile(), gnss_raw::kRxPx1105r, "PX1105R"},
                 {gnss_raw::genericProfile(), gnss_raw::kRxGeneric, "generic"}};
    for (const auto& s : stubs)
    {
        SCOPED_TRACE(s.name);
        EXPECT_FALSE(s.p.measured);
        EXPECT_EQ(s.p.type, s.type);
        EXPECT_STREQ(s.p.name, s.name);
        expectSameNumbers(s.p, lc);
        EXPECT_EQ(gnss_raw::profileFor(s.type).type, s.type);
    }
    EXPECT_EQ(gnss_raw::profileFor(gnss_raw::kRxLc86g).type, gnss_raw::kRxLc86g);
    EXPECT_TRUE(gnss_raw::profileFor(gnss_raw::kRxLc86g).measured);
}

TEST(ReceiverProfile, ThePhaseSigmaIsLogLinearInCn0)
{
    const GnssSignalProfile gps = gnss_raw::lc86gProfile().sig[GNSS_SYS_GPS];
    const GnssSignalProfile gal = gnss_raw::lc86gProfile().sig[GNSS_SYS_GAL];
    // the anchors
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gps, 35.0f), 0.0045f, 1e-7f);
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gps, 45.0f), 0.0014f, 1e-7f);
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gal, 35.0f), 0.0024f, 1e-7f);
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gal, 45.0f), 0.0011f, 1e-7f);
    // halfway in dB: the anchors' geometric mean
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gps, 40.0f), std::sqrt(0.0045f * 0.0014f), 1e-7f);
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gal, 40.0f), std::sqrt(0.0024f * 0.0011f), 1e-7f);
    // held above 45
    EXPECT_EQ(gnss_raw::phaseSigmaM(gps, 52.0f), 0.0014f);
    // extrapolated below 35, on the anchors' own slope -- GPS's close to
    // 1/sqrt(C/N0)'s (3.2 over 10 dB for its 3.16), so 5 dB down is about 10^(5/20) up
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gps, 30.0f), 0.0045f * std::pow(0.0045f / 0.0014f, 0.5f), 1e-7f);
    EXPECT_NEAR(gnss_raw::phaseSigmaM(gps, 30.0f), 0.0045f * std::pow(10.0f, 0.25f), 1e-4f);
    // monotone
    float last = 1.0f;
    for (float c = 20.0f; c <= 50.0f; c += 0.5f)
    {
        const float s = gnss_raw::phaseSigmaM(gps, c);
        EXPECT_LE(s, last) << c;
        last = s;
    }
    // no anchors: no sigma
    EXPECT_EQ(gnss_raw::phaseSigmaM(GnssSignalProfile{}, 40.0f), 0.0f);
}

// ---- a receiver fused faster than 1 Hz (the mosaic-G5, fc/v9-raw-primary)

TEST(ReceiverProfile, TheLc86gFusesEveryEpochOnItsOwnCounts)
{
    // the LC86G's new numbers are what the filter flies without them: every
    // epoch fused, each safeguard on its count, its epoch ages the config's
    const GnssReceiverProfile p = gnss_raw::lc86gProfile();
    EXPECT_EQ(p.fuse_epoch_period_s, 0.0);
    EXPECT_EQ(p.pr_period_s, 0.0);
    EXPECT_EQ(p.sat_cap, 0);
    EXPECT_EQ(p.reject_persist_s, 0.0);
    EXPECT_EQ(p.level_min_dt_s, 0.0);
    EXPECT_EQ(p.heading_fit_period_s, 0.0);
    EXPECT_EQ(p.rr_dyn_window_s, 0.0);
    EXPECT_EQ(p.max_epoch_age_s, 0.6);
    EXPECT_EQ(p.epoch_job_max_s, 0.3);
    RawMeasBuilder::Config b;
    gnss_raw::applyReceiverProfile(p, b);
    EXPECT_EQ(b.epoch_period_s, 1.0);                     // its raw epochs' period
    RawGnssEKFParams e;
    e.reject_persist = 4;                                 // a bench's: no reject_persist_s, kept
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist, 4);
    EXPECT_EQ(e.reject_persist_rr, -1);
    RawGnssShadow::Config s;
    gnss_raw::applyReceiverProfile(p, s);
    EXPECT_EQ(s.max_epoch_age_s, 0.6);
    EXPECT_EQ(s.epoch_job_max_s, 0.3);
    EXPECT_EQ(s.pr_period_s, 0.0);
    EXPECT_EQ(s.level.min_dt_s, 0.0);
}

TEST(ReceiverProfile, TheMosaicG5V0Profile)
{
    // every number of v0 (ReceiverProfile.h: each one's run beside it)
    const GnssReceiverProfile p = gnss_raw::mosaicG5Profile();
    EXPECT_STREQ(p.name, "mosaic-G5");
    EXPECT_EQ(p.type, gnss_raw::kRxMosaicG5);
    EXPECT_FALSE(p.measured);                             // not in flight
    EXPECT_EQ(gnss_raw::profileFor(gnss_raw::kRxMosaicG5).type, gnss_raw::kRxMosaicG5);
    // signals: L1 C/A, E1, B1I (and QZSS as GPS), the code and Doppler floored
    // at 25 dB-Hz; the carrier's as the LC86G's (off)
    const GnssReceiverProfile lc = gnss_raw::lc86gProfile();
    const double hz[4] = {GNSS_FREQ_L1_HZ, GNSS_FREQ_L1_HZ, 1561.098e6, GNSS_FREQ_L1_HZ};
    for (int k = 0; k < gnss_raw::kProfileSys; ++k)
    {
        SCOPED_TRACE(testing::Message() << "sys " << k);
        EXPECT_EQ(p.sig[k].carrier_hz, hz[k]);
        EXPECT_EQ(p.sig[k].cn0_min_pr, 25.0f);
        EXPECT_EQ(p.sig[k].cn0_min_rr, 25.0f);
        EXPECT_EQ(p.sig[k].cn0_min_cr, lc.sig[k].cn0_min_cr);
        EXPECT_EQ(p.sig[k].phase_sigma35_m, lc.sig[k].phase_sigma35_m);
        EXPECT_EQ(p.sig[k].phase_sigma45_m, lc.sig[k].phase_sigma45_m);
        EXPECT_EQ(p.sig[k].half_cycle_unflagged, lc.sig[k].half_cycle_unflagged);
    }
    // timing
    EXPECT_EQ(p.rr_lag_s, 0.0);
    EXPECT_EQ(p.epoch_latency_s, 0.0);
    EXPECT_EQ(p.raw_epoch_period_s, 0.05);
    EXPECT_EQ(p.fuse_epoch_period_s, 0.1);
    // the code
    EXPECT_EQ(p.pr_white_a, 0.10);
    EXPECT_EQ(p.pr_white_b, 0.30);
    EXPECT_EQ(p.pr_corr_a, 1.94);
    EXPECT_EQ(p.pr_corr_b, 3.5);
    EXPECT_EQ(p.pr_corr_scale, 0.5);
    EXPECT_EQ(p.pr_tau_s, 15.0);
    EXPECT_EQ(p.pr_period_s, 1.0);
    // a whole number of fused epochs: the code on the same epoch of each second
    EXPECT_EQ(std::llround(p.pr_period_s * 1000.0) % std::llround(p.fuse_epoch_period_s * 1000.0), 0);
    // the Doppler
    EXPECT_EQ(p.rr_white_a, 0.01);
    EXPECT_EQ(p.rr_white_b, 0.075);
    EXPECT_EQ(p.rr_scale, 1.5);
    EXPECT_EQ(p.rr_tau_s, 0.0);
    EXPECT_EQ(p.doppler_step_hz, 0.0);
    EXPECT_EQ(p.rr_dyn_sigma, 0.5);
    EXPECT_EQ(p.rr_dyn_window_s, 1.0);
    EXPECT_EQ(p.rr_relock_sigma, 3.0);
    // a relock
    EXPECT_EQ(p.relock_sigma_m, 40.0);
    EXPECT_EQ(p.relock_tau_s, 10.0);
    EXPECT_EQ(p.relock_gap_s, 0.5);
    // the clock: the LC86G's, its drift's walk 100
    EXPECT_EQ(p.clk_bias_psd, 0.009);
    EXPECT_EQ(p.clk_drift_psd, 100.0);
    EXPECT_EQ(p.isb_psd, 1.0e-4);
    EXPECT_EQ(p.clock_step_m, 1000.0);
    EXPECT_EQ(p.clock_step_spread_m, 200.0);
    EXPECT_EQ(p.drift_step_mps, 10.0);
    EXPECT_EQ(p.drift_step_spread_mps, 2.0);
    EXPECT_EQ(p.p0_clk_bias, 30.0);
    EXPECT_EQ(p.p0_clk_drift, 5.0);
    EXPECT_EQ(p.p0_isb, 30.0);
    // the carrier off (the rest of it the LC86G's)
    EXPECT_EQ(p.cr_mode, gnss_raw::kCarrierOff);
    EXPECT_EQ(p.cr_min_lock_ms, lc.cr_min_lock_ms);
    EXPECT_EQ(p.cr_max_interval_s, lc.cr_max_interval_s);
    EXPECT_EQ(p.cr_sigma_floor_mps, lc.cr_sigma_floor_mps);
    EXPECT_EQ(p.cr_sigma_per_rev_s, lc.cr_sigma_per_rev_s);
    EXPECT_EQ(p.cr_common_sigma_mps, lc.cr_common_sigma_mps);
    EXPECT_EQ(p.cr_screen_k, lc.cr_screen_k);
    EXPECT_EQ(p.cr_screen_floor_mps, lc.cr_screen_floor_mps);
    EXPECT_EQ(p.cr_half_cycle, lc.cr_half_cycle);
    // the limits
    EXPECT_EQ(p.sat_cap, 16);
    EXPECT_EQ(p.reject_persist_s, 3.0);
    EXPECT_EQ(p.max_epoch_age_s, 0.3);
    EXPECT_EQ(p.epoch_job_max_s, 0.2);
    EXPECT_EQ(p.level_min_dt_s, 0.95);
    EXPECT_EQ(p.heading_fit_period_s, 1.0);

    // applied: the builder's epochs the fused grid's apart, no carrier
    RawMeasBuilder::Config b;
    gnss_raw::applyReceiverProfile(p, b);
    EXPECT_EQ(b.epoch_period_s, 0.1);
    EXPECT_FALSE(b.carrier);
    EXPECT_EQ(b.relock_gap_s, 0.5);
    EXPECT_EQ(b.rr_tau_s, 0.0);
    // the EKF's: the Beetle's 3 s of refusals, the Doppler's 30 epochs at 10
    // Hz, the pseudorange's 3 at its 1 Hz
    RawGnssEKFParams e;
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist, 3);
    EXPECT_EQ(e.reject_persist_rr, 30);
    EXPECT_EQ(e.rr_lag_s, 0.0);
    EXPECT_EQ(e.cr_mode, gnss_raw::kCarrierOff);
    EXPECT_FALSE(e.att_gnss_off);                         // the harness's switch, never a profile's
    // the shadow's
    RawGnssShadow::Config s;
    gnss_raw::applyReceiverProfile(p, s);
    EXPECT_EQ(s.epoch_latency_s, 0.0);
    EXPECT_EQ(s.max_epoch_age_s, 0.3);
    EXPECT_EQ(s.epoch_job_max_s, 0.2);
    EXPECT_EQ(s.pr_period_s, 1.0);
    EXPECT_EQ(s.rr_dyn_window_s, 1.0);
    EXPECT_EQ(s.heading_fit_period_s, 1.0);
    EXPECT_EQ(s.sat_cap, 16);
    EXPECT_EQ(s.level.min_dt_s, 0.95);
    EXPECT_EQ(s.meas.epoch_period_s, 0.1);
    EXPECT_EQ(s.ekf.reject_persist_rr, 30);
    // what main.cpp sets on the mosaic build, not the profile
    EXPECT_FALSE(s.level_on_fused_epochs);
    EXPECT_EQ(s.restore_pos_sigma_m, 0.0);
    EXPECT_FALSE(s.restore_seed_bias);
}

TEST(ReceiverProfile, ARefusalsLetInIsKeptInTime)
{
    // reject_persist_s in each kind's fused epochs: the Doppler every fused
    // epoch, the pseudorange on its own period; at least one, at most what
    // the EKF's per-satellite count reaches
    GnssReceiverProfile p = gnss_raw::mosaicG5Profile();
    RawGnssEKFParams e;
    p.fuse_epoch_period_s = 0.05;                         // all 20 Hz
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist_rr, 60);
    EXPECT_EQ(e.reject_persist, 3);
    p.fuse_epoch_period_s = 0.2;                          // the ladder's 5 Hz
    p.pr_period_s = 0.0;                                  //   the code every epoch too
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist_rr, 15);
    EXPECT_EQ(e.reject_persist, 15);
    p.fuse_epoch_period_s = 0.0;                          // the raw epochs' own period
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist_rr, 60);
    p.reject_persist_s = 30.0;                            // past the count's 255
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist_rr, 255);
    p.reject_persist_s = 0.01;
    gnss_raw::applyReceiverProfile(p, e);
    EXPECT_EQ(e.reject_persist_rr, 1);
}

// The flight safeguards' fields (applyMosaicFlightSafeguards sets these and
// nothing else): r takes each from b.
void takeFlightSafeguards(RawGnssShadow::Config& r, const RawGnssShadow::Config& b)
{
    r.dv_check = b.dv_check;
    r.dv_window_s = b.dv_window_s;
    r.dv_att_allow_deg = b.dv_att_allow_deg;
    r.dv_epoch_k = b.dv_epoch_k;
    r.dv_latch_k = b.dv_latch_k;
    r.dv_ground = b.dv_ground;
    r.dv_sat_k = b.dv_sat_k;
    r.dv_floor_mps = b.dv_floor_mps;
    r.dv_sigma_per_g = b.dv_sigma_per_g;
    r.dv_scale_allow = b.dv_scale_allow;
    r.dv_bias_allow_mps2 = b.dv_bias_allow_mps2;
    r.dv_lever_allow_m = b.dv_lever_allow_m;
    r.dv_hold_s = b.dv_hold_s;
    r.dv_epoch_hold_s = b.dv_epoch_hold_s;
    r.dv_deweight = b.dv_deweight;
    r.dv_noise_tau_s = b.dv_noise_tau_s;
    r.dv_hold_all_frac = b.dv_hold_all_frac;
    r.ekf.hdg_obs_mode = b.ekf.hdg_obs_mode;
    r.ekf.hdg_obs_fh_mps2 = b.ekf.hdg_obs_fh_mps2;
    r.ekf.hdg_obs_tau_s = b.ekf.hdg_obs_tau_s;
    r.ekf.hdg_obs_open_g = b.ekf.hdg_obs_open_g;
    r.ekf.hdg_obs_open_hold_s = b.ekf.hdg_obs_open_hold_s;
    r.ekf.hdg_obs_floor = b.ekf.hdg_obs_floor;
    r.heading_obs_scope = b.heading_obs_scope;
    r.rot_lever_axial_m = b.rot_lever_axial_m;
    r.rot_lever_radial_m = b.rot_lever_radial_m;
    r.rot_lever_min_rps = b.rot_lever_min_rps;
    r.rot_after_apogee = b.rot_after_apogee;
    r.att_hold_rate_rps = b.att_hold_rate_rps;
    r.att_hold_s = b.att_hold_s;
    r.rot_skip_rate_rps = b.rot_skip_rate_rps;
    r.rot_skip_s = b.rot_skip_s;
}

TEST(ReceiverProfile, TheMosaicFlightSafeguards)
{
    // gnss_raw::applyMosaicFlightSafeguards: every value the mosaic-only image
    // flies its three safeguards at, from one place (main.cpp's startRawShadow
    // under TR_RAW_NAV_MOSAIC, and tools/raw_replay's --receiver mosaic)
    static_assert(std::is_trivially_copyable<RawGnssShadow::Config>::value,
                  "the byte comparison below copies the config whole");
    RawGnssShadow::Config base;
    gnss_raw::applyReceiverProfile(gnss_raw::mosaicG5Profile(), base);
    RawGnssShadow::Config c;
    std::memcpy(&c, &base, sizeof(c));
    gnss_raw::applyMosaicFlightSafeguards(c);
    // the Doppler change check: acting, in flight only, a 1 s window, the
    // epoch at 4 sigmas with a 10 deg attitude allowance, the latch at 5; the
    // rest at the check's own defaults (dv_lever_allow_m a bound, not the
    // antenna's offset)
    EXPECT_EQ(c.dv_check, RawGnssShadow::kDvAct);
    EXPECT_EQ(c.dv_window_s, 1.0);
    EXPECT_EQ(c.dv_att_allow_deg, 10.0);
    EXPECT_EQ(c.dv_epoch_k, 4.0);
    EXPECT_EQ(c.dv_latch_k, 5.0);
    EXPECT_FALSE(c.dv_ground);
    EXPECT_EQ(c.dv_sat_k, 5.0);
    EXPECT_EQ(c.dv_floor_mps, 0.03);
    EXPECT_EQ(c.dv_sigma_per_g, 0.1);
    EXPECT_EQ(c.dv_scale_allow, 0.01);
    EXPECT_EQ(c.dv_bias_allow_mps2, 0.1);
    EXPECT_EQ(c.dv_lever_allow_m, 0.3);
    EXPECT_EQ(c.dv_hold_s, 1.0);
    EXPECT_EQ(c.dv_epoch_hold_s, 0.5);
    EXPECT_FALSE(c.dv_deweight);
    EXPECT_EQ(c.dv_noise_tau_s, 0.0);
    EXPECT_EQ(c.dv_hold_all_frac, 0.5);
    // GNSS's heading gain by its observability: the rows' heading part
    // scaled by the 1 s mean horizontal specific force over 5 m/s^2, open past
    // 2 g within 0.2 s, no floor, everywhere (the pad included)
    EXPECT_EQ(c.ekf.hdg_obs_mode, gnss_raw::kHdgObsForce);
    EXPECT_EQ(c.ekf.hdg_obs_fh_mps2, 5.0);
    EXPECT_EQ(c.ekf.hdg_obs_tau_s, 1.0);
    EXPECT_EQ(c.ekf.hdg_obs_open_g, 2.0);
    EXPECT_EQ(c.ekf.hdg_obs_open_hold_s, 0.2);
    EXPECT_EQ(c.ekf.hdg_obs_floor, 0.0);
    EXPECT_EQ(c.heading_obs_scope, RawGnssShadow::kObsEverywhere);
    // the turning body: off in this step
    EXPECT_EQ(c.rot_lever_axial_m, 0.0);
    EXPECT_EQ(c.rot_lever_radial_m, 0.0);
    EXPECT_EQ(c.rot_lever_min_rps, 0.0);
    EXPECT_FALSE(c.rot_after_apogee);
    EXPECT_EQ(c.att_hold_rate_rps, 0.0);
    EXPECT_EQ(c.att_hold_s, 0.0);
    EXPECT_EQ(c.rot_skip_rate_rps, 0.0);
    EXPECT_EQ(c.rot_skip_s, 0.0);
    // and nothing else: the profile's settings, the builder's, the EKF's, the
    // vertical check (20 m/s), the boost screen and the heading gate as they
    // were, byte for byte once the safeguards' own fields are put back
    RawGnssShadow::Config back;
    std::memcpy(&back, &c, sizeof(back));
    takeFlightSafeguards(back, base);
    EXPECT_EQ(0, std::memcmp(&back, &base, sizeof(back)));
    EXPECT_EQ(c.vert_check_mps, 20.0);
    EXPECT_EQ(c.heading_gate, RawGnssShadow::kGateFullLaunch);
    expectSameShadowSettings(c, base);

    // The Beetle's: its defaults and the LC86G's profile keep all three off
    // (main.cpp never calls the function on M1)
    RawGnssShadow::Config m1;
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), m1);
    for (const RawGnssShadow::Config* d : {&m1, &base})
    {
        EXPECT_EQ(d->dv_check, RawGnssShadow::kDvOff);
        EXPECT_EQ(d->ekf.hdg_obs_mode, gnss_raw::kHdgObsOff);
        EXPECT_EQ(d->rot_lever_axial_m, 0.0);
        EXPECT_EQ(d->rot_lever_radial_m, 0.0);
        EXPECT_EQ(d->rot_lever_min_rps, 0.0);
        EXPECT_EQ(d->att_hold_rate_rps, 0.0);
        EXPECT_EQ(d->rot_skip_rate_rps, 0.0);
    }
    const RawGnssEKFParams params;
    EXPECT_EQ(params.hdg_obs_mode, gnss_raw::kHdgObsOff);
}

TEST(ReceiverProfile, TheShadowSafeguardDefaultsAreTheBeetles)
{
    // Every RawGnssShadow::Config default, literally: the Beetle's navigation
    // filter is these (main.cpp sets primary, min_step_s, epoch_slice_us,
    // external_build and heading_mode over them on M1, and applies the
    // LC86G's profile, which changes none). A new field joins this list and
    // the size beside the comparators.
    const RawGnssShadow::Config c;
    EXPECT_EQ(c.epoch_latency_s, 0.0);
    EXPECT_EQ(c.max_epoch_age_s, 0.6);
    EXPECT_EQ(c.epoch_slice_us, 0.0);
    EXPECT_EQ(c.epoch_job_max_s, 0.3);
    EXPECT_EQ(c.log_period_s, 0.1);
    EXPECT_EQ(c.min_step_s, 0.04);
    EXPECT_EQ(c.resync_period_s, 1.0);
    EXPECT_EQ(c.seed_max_age_s, 2.0);
    EXPECT_EQ(c.max_gap_s, 0.5);
    EXPECT_EQ(c.pad_reset_m, 500.0);
    EXPECT_EQ(c.default_week, 2440);
    EXPECT_EQ(c.iono_max_age_days, 30);
    EXPECT_EQ(c.survey.tau_s, 120.0);
    EXPECT_EQ(c.survey.settle_s, 50.0);
    EXPECT_EQ(c.survey.min_span_s, 30.0);
    EXPECT_EQ(c.survey.gap_s, 2.5);
    EXPECT_EQ(c.survey.max_resid_m, 30.0);
    EXPECT_EQ(c.survey.pad_move_m, 20.0);
    EXPECT_EQ(c.survey.max_pad_offset_m, 30.0);
    EXPECT_EQ(c.survey.iono_zenith_m, 5.0f);
    EXPECT_EQ(c.sat_check.jump_m, 20.0);
    EXPECT_EQ(c.sat_check.min_sats, 5);
    EXPECT_EQ(c.sat_check.max_dt_s, 1.6);
    EXPECT_EQ(c.fix_check.pos_m, 3.0);
    EXPECT_EQ(c.fix_check.vel_mps, 0.6);
    EXPECT_EQ(c.fix_check.jitter_s, 0.02);
    EXPECT_EQ(c.fix_check.time_s, 0.05);
    EXPECT_EQ(c.fix_check.acc_err, 0.5);
    EXPECT_EQ(c.fix_check.acc_frac, 0.035);
    EXPECT_EQ(c.fix_check.max_span_s, 4.0);
    EXPECT_EQ(c.acc_window_s, 0.08);
    EXPECT_EQ(c.level.accel_max_mps2, 0.5);
    EXPECT_EQ(c.level.steady_epochs, 2);
    EXPECT_EQ(c.level.g_tol_mps2, 0.5);
    EXPECT_EQ(c.level.max_gap_s, 2.5);
    EXPECT_EQ(c.level.outage_s, 2.0);
    EXPECT_EQ(c.level.g_hold_s, 5.0);
    EXPECT_EQ(c.level.min_dt_s, 0.0);
    EXPECT_EQ(c.baro_sigma_m, 2.0);
    EXPECT_EQ(c.baro_port_k, 0.0075);
    EXPECT_EQ(c.baro_offset_tau_s, 30.0);
    EXPECT_EQ(c.baro_offset_min_s, 10.0);
    EXPECT_EQ(c.baro_hold_port_m, 2.0);
    EXPECT_EQ(c.baro_hold_max_s, 30.0);
    EXPECT_EQ(c.zupt_sigma_mps, 0.5);
    EXPECT_EQ(c.zupt_period_s, 0.1);
    EXPECT_TRUE(c.pad_zupt);
    EXPECT_EQ(c.still_rate_rps, 0.05);
    EXPECT_FALSE(c.primary);
    EXPECT_EQ(c.mag_field_tol, 0.15);
    EXPECT_EQ(c.mag_dip_tol_deg, 6.0);
    EXPECT_EQ(c.max_bridge_s, 5.0);
    EXPECT_EQ(c.pad_context_delay_s, 6.0);
    EXPECT_FALSE(c.external_build);
    EXPECT_EQ(c.heading_mode, RawGnssShadow::kHeadingOff);
    EXPECT_EQ(c.heading_fit.bias_sigma_mps2, 0.09807);
    EXPECT_EQ(c.heading_fit.drift_mps2, 0.02);
    EXPECT_EQ(c.heading_fit.cut_chi2, 13.8);
    EXPECT_EQ(c.heading_fit.min_speed_mps, 1.0);
    EXPECT_EQ(c.heading_fit.tau_s, 1.0);
    EXPECT_EQ(c.heading_seed_max_s, 3.0);
    EXPECT_EQ(c.heading_fit_max_s, 60.0);
    EXPECT_EQ(c.heading_handoff_deg, -1.0);
    EXPECT_EQ(c.heading_handoff_rival, 25.0);
    EXPECT_EQ(c.heading_handoff_min_epochs, 0);
    EXPECT_EQ(c.heading_handoff_apart, 2.0);
    EXPECT_EQ(c.heading_handoff_sure_deg, 3.0);
    EXPECT_EQ(c.heading_pad_sigma_deg, 10.0);
    EXPECT_EQ(c.heading_pad_sigma_fallback_deg, 30.0);
    EXPECT_EQ(c.heading_mag_recent_s, 5.0);
    EXPECT_EQ(c.heading_launch_tilt_sigma_deg, 0.6);
    EXPECT_EQ(c.heading_gate, RawGnssShadow::kGateFullLaunch);
    EXPECT_EQ(c.heading_handoff_vel_mps, 3.0);
    EXPECT_EQ(c.heading_obs_scope, RawGnssShadow::kObsEverywhere);   // where the EKF's hdg_obs_mode (off) applies
    EXPECT_EQ(c.boost_screen, RawGnssShadow::kScreenAct);
    EXPECT_EQ(c.boost_screen_g, 2.0);
    EXPECT_EQ(c.boost_screen_window_s, 1.5);
    EXPECT_EQ(c.vert_check_mps, 20.0);
    EXPECT_EQ(c.vert_release_mps, 5.0);
    // the Doppler change check: off (exp/v9-dvcheck); its settings act only when it is on
    EXPECT_EQ(c.dv_check, RawGnssShadow::kDvOff);
    EXPECT_EQ(c.dv_sat_k, 5.0);
    EXPECT_EQ(c.dv_epoch_k, 5.0);
    EXPECT_EQ(c.dv_floor_mps, 0.03);
    EXPECT_EQ(c.dv_sigma_per_g, 0.1);
    EXPECT_EQ(c.dv_att_allow_deg, 30.0);
    EXPECT_EQ(c.dv_scale_allow, 0.01);
    EXPECT_EQ(c.dv_bias_allow_mps2, 0.1);
    EXPECT_EQ(c.dv_lever_allow_m, 0.3);
    EXPECT_EQ(c.dv_hold_s, 1.0);
    EXPECT_EQ(c.dv_epoch_hold_s, 0.5);
    EXPECT_EQ(c.dv_window_s, 0.0);
    EXPECT_FALSE(c.dv_deweight);
    EXPECT_FALSE(c.dv_ground);
    EXPECT_EQ(c.dv_noise_tau_s, 0.0);
    EXPECT_EQ(c.dv_hold_all_frac, 0.5);
    EXPECT_EQ(c.dv_latch_k, 0.0);
    // a receiver fused faster than 1 Hz, and a restored flight: all off
    EXPECT_EQ(c.pr_period_s, 0.0);
    EXPECT_EQ(c.rr_dyn_window_s, 0.0);
    EXPECT_EQ(c.heading_fit_period_s, 0.0);
    EXPECT_FALSE(c.level_on_fused_epochs);
    EXPECT_EQ(c.sat_cap, 0);
    EXPECT_EQ(c.restore_pos_sigma_m, 0.0);
    EXPECT_EQ(c.restore_vel_sigma_mps, 0.0);
    EXPECT_EQ(c.restore_att_sigma_deg, 0.0);
    EXPECT_FALSE(c.restore_seed_bias);
    // the turning body's allowances: off
    EXPECT_EQ(c.rot_lever_axial_m, 0.0);
    EXPECT_EQ(c.rot_lever_radial_m, 0.0);
    EXPECT_EQ(c.rot_lever_min_rps, 0.0);
    EXPECT_EQ(c.att_hold_rate_rps, 0.0);
    EXPECT_EQ(c.att_hold_s, 0.0);
    EXPECT_EQ(c.rot_skip_rate_rps, 0.0);
    EXPECT_EQ(c.rot_skip_s, 0.0);
    EXPECT_FALSE(c.rot_after_apogee);

    // its EKF's, literally (the profile's part is the LC86G's, above)
    const RawGnssEKFParams& e = c.ekf;
    EXPECT_EQ(e.a_noise, 0.20);
    EXPECT_EQ(e.a_markov_sigma, 0.01);
    EXPECT_EQ(e.a_markov_tau, 100.0);
    EXPECT_EQ(e.w_noise, 0.00175);
    EXPECT_EQ(e.w_markov_sigma, 0.00025);
    EXPECT_EQ(e.w_markov_tau, 50.0);
    EXPECT_EQ(e.accel_var, 0.25);
    EXPECT_TRUE(e.level_heading_free);
    EXPECT_EQ(e.mag_sigma_ut, 0.5);
    EXPECT_EQ(e.mag_tilt_sigma_mps2, 0.05);
    EXPECT_EQ(e.mag_r_floor, 0.0027);
    EXPECT_EQ(e.mag_r_ceiling, 0.06);
    EXPECT_EQ(e.mag_bias_nis, 9.0);
    EXPECT_EQ(e.gate_chi2, 10.83);
    EXPECT_EQ(e.reject_persist, 3);
    EXPECT_EQ(e.reject_persist_rr, -1);
    EXPECT_TRUE(e.att_gate_cos4);
    EXPECT_FALSE(e.att_gate_vertical);
    EXPECT_FALSE(e.att_gnss_off);
    // the heading's share by its observability (exp/v9-observ): off, and its settings
    EXPECT_EQ(e.hdg_obs_mode, gnss_raw::kHdgObsOff);
    EXPECT_EQ(e.hdg_obs_fh_mps2, 2.0);
    EXPECT_EQ(e.hdg_obs_tau_s, 1.0);
    EXPECT_EQ(e.hdg_obs_sigma_v, 0.1);
    EXPECT_EQ(e.hdg_obs_window_s, 3.0);
    EXPECT_EQ(e.hdg_obs_open_g, 2.0);
    EXPECT_EQ(e.hdg_obs_open_hold_s, 0.2);
    EXPECT_EQ(e.hdg_obs_floor, 0.0);
    EXPECT_EQ(e.p0_pos, 10.0);
    EXPECT_EQ(e.p0_vel, 1.0);
    EXPECT_EQ(e.p0_att, 0.34906);
    EXPECT_EQ(e.p0_hdg, 0.5);
    EXPECT_EQ(e.p0_abias, 0.981);
    EXPECT_EQ(e.p0_wbias, 0.01745);
    EXPECT_FALSE(e.gravity_wgs84);
    EXPECT_EQ(e.hist_spacing_s, 0.004);
    RawGnssEKFParams lc;
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), lc);
    expectSameEkf(e, lc);
    RawMeasBuilder::Config lb;
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), lb);
    expectSameBuilder(c.meas, lb);
}

TEST(ReceiverProfile, TheRawEpochAndTheEphemerisStoreHoldTheBeetles)
{
    // the Beetle's capacities: a raw epoch's 48 observations, the store's 64
    // ephemerides (two each for 32 satellites) on a default build
    EXPECT_EQ(gnss_raw::RawEpoch::kMaxObs, 48);
    EXPECT_EQ(gnss_raw::EphemerisStore::kCapacity, 64u);
    EXPECT_EQ(gnss_raw::EphemerisStore::kPerSat, 2);
}
