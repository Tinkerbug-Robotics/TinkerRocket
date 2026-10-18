// ReceiverProfile.h — what the raw-GNSS filter takes from the receiver it has
// (#1600): every gate, threshold, noise model and feature mode that is the
// receiver's behaviour rather than physics, in one place, chosen by the
// board's receiver and overridable per build.
//
// The Beetle's LC86G is the one measured. The mosaic-G5's is v0: its numbers
// from the receiver's own logs on the V9 spin rig and the COCOM rig, still
// UNMEASURED in flight. The NEO-M8T and PX1105R profiles are stubs, the
// LC86G's numbers as placeholders, marked UNMEASURED: a board that carries one
// starts from them and measures its own. The generic profile is the same
// placeholders for a receiver with none.
//
// One function puts a profile into the filter's configs (RawGnssShadow's
// ReceiverProfileApply.h: the shadow's, the builder's and the EKF's). The
// configs' own defaults are the LC86G profile's values, so a default-
// constructed builder or filter is the LC86G's (tests_cpp pins the two
// together).
//
// The carrier range rate (cr_*). The change in a satellite's carrier phase
// over an epoch, as a range rate: millimetres a second where the Doppler is
// centimetres to decimetres (RawMeasBuilder builds it, TR_RawGnssEKF screens
// and fuses it). A new feature: it starts in observe (computed, screened and
// counted against the filter, never fused) and is fused where data shows a
// gain -- a profile setting.
//
// Header-only and pure: the profiles are built where they are asked for (on
// the flight computer, once at start, in flash), not in the builder's archive
// (TR_GNSSRaw runs from internal RAM on the Beetle, raw_gnss_iram.lf).
#ifndef GNSS_RECEIVER_PROFILE_H
#define GNSS_RECEIVER_PROFILE_H

#include <cmath>
#include <cstdint>

#include "gnss/types.h"

namespace gnss_raw
{

// The receivers a profile is for.
enum GnssReceiverType : uint8_t
{
    kRxGeneric = 0,
    kRxLc86g = 1,      // Quectel LC86G (the Beetle)
    kRxNeoM8t = 2,     // u-blox NEO-M8T
    kRxPx1105r = 3,    // SkyTraq PX1105R
    kRxMosaicG5 = 4,   // Septentrio mosaic-G5
};

// What the carrier range rates are for.
enum CarrierMode : uint8_t
{
    kCarrierOff = 0,       // not built
    kCarrierObserve = 1,   // built, screened and counted against the filter; the state never moves
    kCarrierFuse = 2,      // and the screened ones fused
};

// What the screen does with a rate a half-cycle step explains.
enum HalfCycleAction : uint8_t
{
    kHalfCycleReject = 0,  // drop it
    kHalfCycleRepair = 1,  // take the half cycle back out and keep it
};

// gnss_sys_t's systems: GPS, Galileo, BeiDou, QZSS.
constexpr int kProfileSys = GNSS_SYS_COUNT;

// One system's signal as the receiver tracks it.
struct GnssSignalProfile
{
    double carrier_hz = GNSS_FREQ_L1_HZ;  // its carrier: the wavelength (a half-cycle step, the Doppler's
                                          // resolution) and the ionosphere's scale
    float  cn0_min_pr = 0.0f;             // dB-Hz: a pseudorange under this is not used (0: no floor)
    float  cn0_min_rr = 0.0f;             //   a Doppler
    float  cn0_min_cr = 0.0f;             //   a carrier phase, at either end of a carrier range rate
    float  phase_sigma35_m = 0.0f;        // the carrier phase's 1-sigma at 35 dB-Hz, m
    float  phase_sigma45_m = 0.0f;        //   and at 45 dB-Hz (phaseSigmaM)
    bool   half_cycle_unflagged = true;   // a half-cycle step can come with its flag (DF420) clear: a data
                                          // channel's Costas loop, not a pilot's PLL
};

struct GnssReceiverProfile
{
    GnssReceiverType type = kRxGeneric;
    const char*      name = "generic";
    bool             measured = false;    // false: UNMEASURED, every number a placeholder

    GnssSignalProfile sig[kProfileSys];   // by gnss_sys_t

    // timing
    double rr_lag_s = 0.0;                // the Doppler is the range rate this long before the epoch's tag
    double epoch_latency_s = 0.0;         // the delay the receiver's time sync leaves in
    double raw_epoch_period_s = 0.0;      // its raw epochs come this often

    // Its epochs as the filter fuses them. A receiver faster than the LC86G's
    // once a second is fused on a coarser grid, and the safeguards that count
    // epochs keep the time they had at 1 Hz (RawGnssShadow.h, "A receiver
    // fused faster than 1 Hz"). 0: every epoch, as the LC86G's.
    double fuse_epoch_period_s = 0.0;     // the GNSS task builds only the epochs on this GPS-time grid
    double pr_period_s = 0.0;             // the pseudoranges fused once in each this long of GPS time (the
                                          //   Dopplers every epoch)
    int    sat_cap = 0;                   // at most this many satellites built an epoch (0: all)
    double reject_persist_s = 0.0;        // a satellite refused this long running is let back in: the EKF's
                                          //   reject_persist and reject_persist_rr in fused epochs (0: theirs)
    double level_min_dt_s = 0.0;          // the levelling's steady test takes epochs this far apart (LevelGate)
    double heading_fit_period_s = 0.0;    // the heading fit takes an epoch at most this often
    double rr_dyn_window_s = 0.0;         // the Doppler's stress on the IMU's peak over this long (0: since the
                                          //   epoch before)
    double max_epoch_age_s = 0.6;         // an epoch later than this is dropped (RawGnssShadow)
    double epoch_job_max_s = 0.3;         //   and one this old is worked at once, not a slice a step
    // The receiver's epochs between the fused ones: each fused epoch's Doppler
    // averaged with the one raw_epoch_period_s before it, where the two are a
    // pair (the mosaic's MosaicRawEpoch.h says when), and the filter taking the
    // pairs' means at their middle (the filter's rr_pair_lag_s: half
    // raw_epoch_period_s more than rr_lag_s, on an epoch handed over in pairs;
    // one handed over as it came keeps rr_lag_s). Only where the fused grid is
    // exactly two of the receiver's epochs (dopplerPairsFit); the sigmas stay
    // the profile's. Off: every receiver.
    bool   rr_pair_avg = false;
    double rr_pair_max_diff_mps = 15.0;   // the pair at most this far apart (300 m/s^2 over 50 ms)

    // its measurements' noise (RawMeasBuilder): sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10)
    double pr_white_a = 0.0, pr_white_b = 0.0;   // the pseudorange's white part, m
    double pr_corr_a = 0.0, pr_corr_b = 0.0;     //   its time-correlated part (first-order Gauss-Markov), m
    double pr_corr_scale = 0.0;                  //   that part scaled
    double pr_tau_s = 0.0;                       //   its correlation time
    double rr_white_a = 0.0, rr_white_b = 0.0;   // the Doppler's, m/s
    double rr_scale = 0.0;
    double rr_tau_s = 0.0;
    double doppler_step_hz = 0.0;                // the Doppler's resolution, whose rounding joins its sigma
    // a relock (a satellite missing longer than relock_gap_s): the pseudorange
    // relock_sigma_m more, decaying over relock_tau_s, the Doppler rr_relock_sigma;
    // a tracking loop under stress: the Doppler rr_dyn_sigma per g above 1 g
    double relock_sigma_m = 0.0, relock_tau_s = 0.0, relock_gap_s = 0.0;
    double rr_dyn_sigma = 0.0, rr_relock_sigma = 0.0;

    // its clock (RawGnssEKFParams): white frequency (bias) and random-walk
    // frequency (drift) noise, the system offsets' walk; a step every satellite
    // agrees on; and the start's sigmas
    double clk_bias_psd = 0.0, clk_drift_psd = 0.0, isb_psd = 0.0;
    double clock_step_m = 0.0, clock_step_spread_m = 0.0;
    double drift_step_mps = 0.0, drift_step_spread_mps = 0.0;
    double p0_clk_bias = 0.0, p0_clk_drift = 0.0, p0_isb = 0.0;

    // the carrier range rate
    uint8_t  cr_mode = kCarrierOff;       // CarrierMode
    uint32_t cr_min_lock_ms = 0;          // both ends' lock time at least this (and the interval)
    double   cr_max_interval_s = 0.0;     // the epochs at most this far apart (and more than half a period)
    double   cr_sigma_floor_mps = 0.0;    // the rate's 1-sigma at least this
    double   cr_sigma_per_rev_s = 0.0;    // and this much more per rev/s of roll (the gyro's), in quadrature
    double   cr_common_sigma_mps = 0.0;   // the epoch's common rate: its prior 1-sigma
    double   cr_screen_k = 0.0;           // the screen: a rate off the epoch's median by more than this
    double   cr_screen_floor_mps = 0.0;   //   many sigmas, and this much, is suspect
    uint8_t  cr_half_cycle = kHalfCycleReject;   // HalfCycleAction
};

// Whether a profile's Dopplers can go in pairs: its fused grid exactly two of
// the receiver's epochs (the mosaic's 100 ms over its 50), so each fused
// epoch has one of the receiver's between it and the last.
inline bool dopplerPairsFit(const GnssReceiverProfile& p)
{
    return p.raw_epoch_period_s > 0.0 &&
           std::fabs(p.fuse_epoch_period_s - 2.0 * p.raw_epoch_period_s) < 1.0e-3 * p.raw_epoch_period_s;
}

// The carrier phase's 1-sigma (m) at cn0 (dB-Hz): interpolated in dB --
// log-linear between the 35 and 45 dB-Hz anchors, as a thermal-noise sigma
// goes as 1/sqrt(C/N0) (the LC86G's GPS anchors, 4.5 and 1.4 mm, are close
// to that law's 10 dB step: 3.2 for its 3.16) -- held at the 45 dB-Hz value
// above it, extrapolated below 35 on the anchors' slope.
inline float phaseSigmaM(const GnssSignalProfile& s, float cn0)
{
    if (!(s.phase_sigma35_m > 0.0f) || !(s.phase_sigma45_m > 0.0f))
        return 0.0f;
    if (cn0 >= 45.0f)
        return s.phase_sigma45_m;
    return s.phase_sigma35_m * std::pow(s.phase_sigma45_m / s.phase_sigma35_m, (cn0 - 35.0f) / 10.0f);
}

// The Quectel LC86G (the Beetle, U5). Each number's source:
//   "existing default": the value the filter flew before profiles (its own
//     measurement noted beside it);
//   "spin rig 2026-10-15": the Beetle itself on the spin rig, static and
//     turning at 0.25-1.5 rev/s;
//   "rig r21/r23/r25": rig replays against truth (coast, drogue, main);
//   "sim (cloud session)": the carrier simulation.
inline GnssReceiverProfile lc86gProfile()
{
    GnssReceiverProfile p;
    p.type = kRxLc86g;
    p.name = "LC86G";
    p.measured = true;

    // One signal per system: GPS L1 C/A, Galileo E1, BeiDou B1I (MSM signal 2,
    // Lc86RawEpoch.h), QZSS L1 C/A (not decoded yet). No C/N0 floor on the code
    // or the Doppler (existing default: none). The carrier's floors: 32.5 dB-Hz
    // (rig r21/r23/r25), Galileo's 35 (rig r21/r23/r25). The phase's sigma
    // anchors (rig r21/r23/r25). A half-cycle step unflagged on the data
    // channels -- GPS, BeiDou, QZSS -- not on Galileo's E1 pilot (spin rig
    // 2026-10-15).
    p.sig[GNSS_SYS_GPS] = GnssSignalProfile{GNSS_FREQ_L1_HZ, 0.0f, 0.0f, 32.5f, 0.0045f, 0.0014f, true};
    p.sig[GNSS_SYS_GAL] = GnssSignalProfile{GNSS_FREQ_L1_HZ, 0.0f, 0.0f, 35.0f, 0.0024f, 0.0011f, false};
    p.sig[GNSS_SYS_BDS] = GnssSignalProfile{1561.098e6, 0.0f, 0.0f, 32.5f, 0.0045f, 0.0014f, true};
    p.sig[GNSS_SYS_QZS] = GnssSignalProfile{GNSS_FREQ_L1_HZ, 0.0f, 0.0f, 32.5f, 0.0045f, 0.0014f, true};

    // timing
    p.rr_lag_s = 0.0555;            // existing default (MSM7 against the rig's truth, boost and coast, 2026-10-12)
    p.epoch_latency_s = 0.0;        // existing default (uncalibrated)
    p.raw_epoch_period_s = 1.0;     // existing default (MSM7 once a second at any fix rate, 2026-09-25..27)

    // its epochs as fused: every one, each safeguard on its count -- all
    // existing default
    p.fuse_epoch_period_s = 0.0;
    p.pr_period_s = 0.0;
    p.sat_cap = 0;
    p.reject_persist_s = 0.0;       // the EKF's 3 epochs: 3 s
    p.level_min_dt_s = 0.0;
    p.heading_fit_period_s = 0.0;
    p.rr_dyn_window_s = 0.0;        // the peak since the epoch before: a second
    p.max_epoch_age_s = 0.6;
    p.epoch_job_max_s = 0.3;

    // noise: the PX1105R's C/N0 fit (20 min static, 2026-09-26), the LC86G's
    // scale on it (COCOM rig, 2026-10-12) -- all existing default
    p.pr_white_a = 0.026;
    p.pr_white_b = 0.157;
    p.pr_corr_a = 1.94;
    p.pr_corr_b = 3.5;
    p.pr_corr_scale = 0.5;          // its correlated pseudorange error half the PX1105R's
    p.pr_tau_s = 15.0;
    p.rr_white_a = 0.0;
    p.rr_white_b = 0.2;
    p.rr_scale = 1.0;
    p.rr_tau_s = 0.1;
    p.doppler_step_hz = 0.0;        // its Doppler is not rounded to a step (SkyTraq's: 1 Hz)
    p.relock_sigma_m = 40.0;        // existing default
    p.relock_tau_s = 10.0;          // existing default
    p.relock_gap_s = 2.5;           // existing default: 2.5 epochs
    p.rr_dyn_sigma = 0.5;           // existing default: m/s per g above 1 g
    p.rr_relock_sigma = 3.0;        // existing default

    // the clock: it steers to its own solution, in steps (+124 m/s of drift
    // within seconds of a 15 g ignition; -2629 km and -71 m/s at its first fix)
    p.clk_bias_psd = 0.009;         // existing default, m^2/s
    p.clk_drift_psd = 100.0;        // existing default, m^2/s^3 (a free TCXO's: 0.036)
    p.isb_psd = 1.0e-4;             // existing default
    p.clock_step_m = 1000.0;        // existing default
    p.clock_step_spread_m = 200.0;  // existing default
    p.drift_step_mps = 10.0;        // existing default
    p.drift_step_spread_mps = 2.0;  // existing default
    p.p0_clk_bias = 30.0;           // existing default
    p.p0_clk_drift = 5.0;           // existing default
    p.p0_isb = 30.0;                // existing default

    // the carrier range rate
    p.cr_mode = kCarrierObserve;    // a new feature: observed until data shows a gain
    p.cr_min_lock_ms = 1000;        // a lock at least an epoch old at both ends (sim (cloud session))
    p.cr_max_interval_s = 1.5;      // consecutive epochs only (one a second)
    p.cr_sigma_floor_mps = 0.008;   // spin rig 2026-10-15 (static: 8.8 mm/s; rig r21/r23/r25: 5.5-12)
    p.cr_sigma_per_rev_s = 0.023;   // spin rig 2026-10-15 (25 mm/s at 1 rev/s)
    p.cr_common_sigma_mps = 5.0;    // sim (cloud session): clock white FM ~0.1 m/s a second, the common wind-up
    p.cr_screen_k = 4.0;            // sim (cloud session): a 3-sigma gate missed 23-27 % of half-cycle steps
    p.cr_screen_floor_mps = 0.03;   // sim (cloud session)
    p.cr_half_cycle = kHalfCycleReject;   // sim (cloud session)
    return p;
}

// The stubs: UNMEASURED. Every number is the LC86G's, a placeholder until the
// receiver is measured on a board that carries it.
inline GnssReceiverProfile placeholderProfile(GnssReceiverType type, const char* name)
{
    GnssReceiverProfile p = lc86gProfile();
    p.type = type;
    p.name = name;
    p.measured = false;
    return p;
}

// u-blox NEO-M8T: UNMEASURED (its UBX-RXM-RAWX would carry the phase).
inline GnssReceiverProfile neoM8tProfile() { return placeholderProfile(kRxNeoM8t, "NEO-M8T"); }

// SkyTraq PX1105R: UNMEASURED. (The base C/N0 noise model is this receiver's
// own fit, 2026-09-26, so its pr_corr_scale is likely 1, not the LC86G's 0.5;
// its Doppler is rounded to 1 Hz, doppler_step_hz.)
inline GnssReceiverProfile px1105rProfile() { return placeholderProfile(kRxPx1105r, "PX1105R"); }

// Septentrio mosaic-G5 (the Goblin, V8/V9 J1), v0: UNMEASURED in flight. Set
// up as the driver sets it (High dynamics, 20 Hz MeasEpoch, no smoothing) and
// fused at 10 Hz, one signal a system, the carrier off. Each number's source:
//   "V9 rig 10/16": the V9, the mosaic and the Goblin's helix on the spin rig,
//     2026-10-16, runs 20261016-140427 (nose cone off) and 20261016-163239
//     (on), the receiver's own SBF;
//   "COCOM rig": the mosaic through the COCOM rig's 10-40 g hotshot (gnss
//     repo, b4989e8 and 1701bbd);
//   "the LC86G's": its number kept, for the reason given;
//   "plan": the 10/24 build plan's choice, the bench's to change (its CPU
//     ladder: 10 Hz x 16 satellites, then 5 x 16, then 5 x 12).
// The rest (the carrier's floors, anchors, flags and intervals) are the
// LC86G's, unused with the carrier off.
inline GnssReceiverProfile mosaicG5Profile()
{
    GnssReceiverProfile p = placeholderProfile(kRxMosaicG5, "mosaic-G5");

    // One signal per system: GPS L1 C/A, Galileo E1 and BeiDou B1I -- its group
    // delay is BDSNav's TGD1, as the RTCM path's (plan); QZSS as GPS (dropped).
    // A C/N0 floor of 25 dB-Hz on the code and the Doppler: below 30 the L1
    // lost lock 9-436 times an hour and its Doppler scattered 0.17-0.18 m/s
    // (V9 rig 10/16); 25 trims only that tail.
    p.sig[GNSS_SYS_GPS].carrier_hz = GNSS_FREQ_L1_HZ;
    p.sig[GNSS_SYS_GAL].carrier_hz = GNSS_FREQ_L1_HZ;
    p.sig[GNSS_SYS_BDS].carrier_hz = 1561.098e6;
    p.sig[GNSS_SYS_QZS].carrier_hz = GNSS_FREQ_L1_HZ;
    for (int k = 0; k < kProfileSys; ++k)
    {
        p.sig[k].cn0_min_pr = 25.0f;
        p.sig[k].cn0_min_rr = 25.0f;
    }

    // timing
    p.rr_lag_s = 0.0;               // V9 rig 10/16: the Doppler against the carrier, 0 +- 2-4 ms on every
                                    //   signal (the LC86G's lags 55.5 ms)
    p.epoch_latency_s = 0.0;        // the 1PPS line: an edge is its GPS second
    p.raw_epoch_period_s = 0.05;    // V9 rig 10/16: every MeasEpoch 50 ms after the last
    p.fuse_epoch_period_s = 0.1;    // plan: 10 Hz, on the GPS-time grid
    p.rr_pair_avg = false;          // the epochs between the grid's averaged in: off until benched
                                    //   (TR_BENCH_RAW_DOP_AVG; the driver's pairs, MosaicRawEpoch.h)

    // the code: sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10)
    p.pr_white_a = 0.10;            // V9 rig 10/16: 0.144 m at 44 dB-Hz, above the receiver's own
    p.pr_white_b = 0.30;            //   L1 C/A CodeVar (0.12-0.13 m)
    p.pr_corr_a = 1.94;             // the LC86G's: mostly the sky's (the ionosphere's residual, the
    p.pr_corr_b = 3.5;              //   broadcast orbits and clocks, multipath), not the receiver's; its
    p.pr_corr_scale = 0.5;          //   PVT agrees (V9 rig 10/16: height sd 1.2 m, HAcc 2.9-3.9 m at 95 %)
    p.pr_tau_s = 15.0;              // the LC86G's, and fused once a second (below): the Beetle's code
    p.pr_period_s = 1.0;            //   weighting and gate exactly (plan)

    // the Doppler
    p.rr_white_a = 0.01;            // V9 rig 10/16: fitted to the L1 C/N0 bins, 176/99/56/32/18 mm/s
    p.rr_white_b = 0.075;           //   against 176/105/57/28/18.6 measured
    p.rr_scale = 1.5;               // plan: a margin for flight vibration and the TCXO's phase noise,
                                    //   neither measured
    p.rr_tau_s = 0.0;               // V9 rig 10/16: white -- lag-1 correlation ~0, lag-2 -0.56 to -0.61
    p.doppler_step_hz = 0.0;        // its resolution is 0.1 mHz
    p.rr_dyn_sigma = 0.5;           // the LC86G's until the hotshot is replayed (COCOM rig: High left
                                    //   0.12-0.26 m/s at 10-40 g, its 4-6 ms lag out)
    p.rr_dyn_window_s = 1.0;        // the LC86G's: its one epoch a second, in time
    p.rr_relock_sigma = 3.0;        // the LC86G's, conservative

    // a relock
    p.relock_sigma_m = 40.0;        // the LC86G's, conservative (the mosaic does no smoothing: its
    p.relock_tau_s = 10.0;          //   relock bias is likely small)
    p.relock_gap_s = 0.5;           // plan: 10 of its epochs (at 2.5, a 150 ms hiccup would relock every
                                    //   satellite and blind the Doppler for 10 s)

    // the clock: the LC86G's, its drift's walk 100 on purpose. Standing still
    // the mosaic's drift walks only 0.001-0.01 m^2/s^3 (V9 rig 10/16), but in
    // flight the TCXO's g-sensitivity, the roll's wind-up and burnout step
    // every Doppler at once: at 0.01 a 1 g step would be refused on every row
    // and the boost screen would drop the boost
    p.clk_bias_psd = 0.009;
    p.clk_drift_psd = 100.0;
    p.isb_psd = 1.0e-4;
    p.clock_step_m = 1000.0;        // its 1 ms jumps the EKF's whole-millisecond branch takes
    p.clock_step_spread_m = 200.0;
    p.drift_step_mps = 10.0;
    p.drift_step_spread_mps = 2.0;
    p.p0_clk_bias = 30.0;
    p.p0_clk_drift = 5.0;           // the first epoch's median sets its 76-86 m/s (V9 rig 10/16)
    p.p0_isb = 30.0;

    // the carrier: off -- at 0.1 s a carrier rate only matches the Doppler;
    // the SBF is logged (0x95) for offline work (plan)
    p.cr_mode = kCarrierOff;

    // the safeguards that count epochs, in the LC86G's time
    p.sat_cap = 16;                 // plan: the CPU ladder's first rung (the L1 sky peaks at 28)
    p.reject_persist_s = 3.0;       // the LC86G's 3 epochs: the Doppler 30 at 10 Hz, the pseudorange 3
    p.max_epoch_age_s = 0.3;        // plan: 3 epochs at 10 Hz (MeasEpoch reaches the V9 20 ms after its
    p.epoch_job_max_s = 0.2;        //   epoch, p95 26 ms: V9 rig 10/16), 2 to work at once
    p.level_min_dt_s = 0.95;        // the LC86G's steady test, epochs a second apart
    p.heading_fit_period_s = 1.0;   // the LC86G's: the fit's 64 epochs a minute (it only logs in act)
    return p;
}

// A receiver with no profile of its own: UNMEASURED.
inline GnssReceiverProfile genericProfile() { return placeholderProfile(kRxGeneric, "generic"); }

// The profile of a receiver type.
inline GnssReceiverProfile profileFor(GnssReceiverType type)
{
    switch (type)
    {
    case kRxLc86g:
        return lc86gProfile();
    case kRxNeoM8t:
        return neoM8tProfile();
    case kRxPx1105r:
        return px1105rProfile();
    case kRxMosaicG5:
        return mosaicG5Profile();
    default:
        return genericProfile();
    }
}

inline const char* carrierModeName(uint8_t mode)
{
    return mode == kCarrierFuse ? "fuse" : (mode == kCarrierObserve ? "observe" : "off");
}

}  // namespace gnss_raw

#endif  // GNSS_RECEIVER_PROFILE_H
