// ReceiverProfile.h — what the raw-GNSS filter takes from the receiver it has
// (#1600): every gate, threshold, noise model and feature mode that is the
// receiver's behaviour rather than physics, in one place, chosen by the
// board's receiver and overridable per build.
//
// The Beetle's LC86G is the one measured. The NEO-M8T, PX1105R and
// mosaic-G5 profiles are stubs, the LC86G's numbers as placeholders, marked
// UNMEASURED: a board that carries one starts from them and measures its own.
// The generic profile is the same placeholders for a receiver with none.
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

// Septentrio mosaic-G5: UNMEASURED.
inline GnssReceiverProfile mosaicG5Profile() { return placeholderProfile(kRxMosaicG5, "mosaic-G5"); }

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
