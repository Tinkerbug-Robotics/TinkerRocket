// Host tests for the receiver profiles (#1600): TR_GNSSRaw/ReceiverProfile.h
// and TR_RawGnssShadow/ReceiverProfileApply.h. The LC86G's profile, applied,
// is exactly what the builder and the filter flew with before there were
// profiles -- and the configs' own defaults, so a default-constructed builder
// or filter is the LC86G's; the stubs are its numbers, marked UNMEASURED; the
// carrier phase's sigma between its C/N0 anchors.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "RawGnssShadow.h"
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
    EXPECT_EQ(a.clock_step_m, b.clock_step_m);
    EXPECT_EQ(a.clock_step_spread_m, b.clock_step_spread_m);
    EXPECT_EQ(a.drift_step_mps, b.drift_step_mps);
    EXPECT_EQ(a.drift_step_spread_mps, b.drift_step_spread_mps);
    EXPECT_EQ(a.rr_lag_s, b.rr_lag_s);
    EXPECT_EQ(a.att_gate_cos4, b.att_gate_cos4);
    EXPECT_EQ(a.att_gate_vertical, b.att_gate_vertical);
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
}

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
                 {gnss_raw::mosaicG5Profile(), gnss_raw::kRxMosaicG5, "mosaic-G5"},
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
