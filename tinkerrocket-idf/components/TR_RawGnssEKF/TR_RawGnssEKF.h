// TR_RawGnssEKF.h — a tightly coupled GNSS/INS filter: the flight filter's
// error states plus the receiver clock, updated by each satellite's
// pseudorange and range rate instead of the receiver's fix (#1600)
//
// The LC86G withholds its own fix from about T-1 s to several seconds after
// ignition and comes back tens of metres off; its raw measurements (RTCM
// MSM7) keep coming throughout. On the COCOM rig (2026-10-12, four flights)
// this filter held altitude to 0.3-3 m and vertical velocity to 0.01-0.4 m/s
// in every phase, where the receiver's fix was 3-91 m and 9-53 m/s off
// (docs/plans/1600-realtime-raw-gnss.md).
//
// A port of the bench prototype that scored it (gnss repo
// sim/tinkerrocket_gnss/tc_ekf.py, TcEkf), with its states, mechanization
// and update rules, in the flight filter's numerical form (TR_GpsInsEKF):
//
//   error state (19)
//     0-2   position, NED (m)
//     3-5   velocity, NED (m/s)
//     6-8   attitude, quaternion vector part (half-angle)
//     9-11  accelerometer bias (m/s^2)
//     12-14 gyro bias (rad/s)
//     15    receiver clock bias (m)
//     16    receiver clock drift (m/s)
//     17    Galileo - GPS time offset (m)
//     18    BeiDou - GPS time offset (m)
//
// Body FRD, world NED, q scalar-first with Quat2DCM(q) = NED->body, as
// TR_GpsInsEKF. The prototype's other states (the clock's g-sensitivity, the
// Doppler's clock-rate offset and lag, the carrier-phase clone) changed
// nothing on the LC86G's flights and are left out; its Doppler lag is a
// constant (rr_lag_s).
//
// Each pseudorange and range rate is a scalar update, gated on its own
// innovation (chi-square, 1 dof), so one bad satellite is dropped without
// losing the rest; a satellite refused reject_persist epochs running is let
// back in with its variance inflated to the gate. A time-correlated
// pseudorange error is fused with its variance scaled by (1+rho)/(1-rho),
// rho = exp(-dt/tau) since that satellite's last update. Before the
// updates, a step every satellite agrees on is taken by the clock: whole
// milliseconds (u-blox, SkyTraq) or not (the LC86G steers its clock to its
// first fix in one step: -2629 km and -71 m/s on the rig). GNSS corrections
// reach the attitude scaled by cos^4 pitch, as the flight filter's.
//
// An epoch can be applied after its time (it reaches the flight computer
// 0.1-0.25 s late): age_s says how long ago it was, and the receiver is
// taken back by the IMU's own velocity increments since (a ring of them,
// ~0.5 s). The Doppler is the range rate rr_lag_s before the epoch's tag
// (55.5 ms on the LC86G), predicted the same way; an epoch whose Dopplers are
// pairs' means (setRrPaired) has them rr_pair_lag_s further back.
//
// The carrier range rate (RawMeas has_cr: a satellite's carrier phase range
// over the interval since its epoch before, RawMeasBuilder), after the
// epoch's pseudoranges and Dopplers, in the receiver profile's mode (off,
// observe, fuse: cr_mode):
//   - the measurement is the interval's MEAN range rate: y = (cr_dphi_m -
//     [rho(s_k, r_k) - rho(s_prev, r_prev)]) / dt - c, both ranges with the
//     Sagnac turn the pseudorange's has. A mid-interval time tag gave 183 mm/s
//     innovations through manoeuvres in the carrier simulation, the mean from
//     the filter's own motion 55 mm/s;
//   - r_k is the receiver at the epoch, back-dated as the pseudoranges take
//     it (beginGnssRaw), and r_prev the one the epoch before left: each a
//     snapshot that every later correction, whatever its source, moves to
//     first order as it moves the trajectory -- dp - dv (t_correction -
//     t_snapshot) -- so r_k - r_prev is the corrected filter's own
//     displacement over the interval, however late the epochs came. Less
//     the integration's excess: the position steps on each step's end
//     velocity, a dt / 2 per second past the velocity's own path (6 cm/s at
//     13 m/s^2 and 10 ms steps, 50 in a 10 g boost) -- invisible to the code,
//     but the carrier would pull the velocity off by it;
//   - c is the epoch's common rate (the receiver clock's white FM, ~0.1 m/s
//     over a second, and the common carrier wind-up), not the clock drift
//     state: a nuisance with the profile's prior about the epoch's median,
//     estimated with the state over the epoch's carrier updates (its
//     cross-covariance P_xc and variance P_cc beside P, Joseph form) and
//     dropped after. H: the Doppler's on the velocity (-u_k: the interval's
//     displacement error is the velocity error times dt), 1 on c. R: the
//     rate's sigma and cr_sigma_per_rev_s per rev/s of roll (the gyro's mean
//     |roll rate| over the interval), in quadrature;
//   - the screen, at the prior: each rate against the others. The epoch's
//     median takes the common rate out, but a Doppler-quality velocity
//     (0.05-0.2 m/s off) puts good rates many sigmas apart along their
//     different lines of sight, so the common rate and a velocity correction
//     are fitted to the rates (the velocity weighted by the filter's own
//     covariance). The worst rate past max(cr_screen_k sigma,
//     cr_screen_floor_mps) on its residual left out of the fit is set aside
//     and the rest refitted, until none is; then each set aside is judged
//     against that clean fit on its own: within it after all, back in; where
//     the system's half cycles can come unflagged, within 3 sigma of
//     lambda/(2 dt) either way, a half-cycle step -- repaired (cr_dphi_m less
//     or plus lambda/2) or dropped, as the profile says; else dropped. Under
//     4 rates there is nothing to screen against: none taken;
//   - observe: the screen, the innovations, the common rate and the velocity
//     correction the rates point to (the fit's), counted, and the state
//     untouched. Fuse: the screened rates fused, through the attitude gate
//     the GNSS updates have.
//
// T is the covariance and attitude/velocity scalar: float on the flight
// computer (single-precision FPU), double for checking the port against the
// prototype. Position (lat/lon/height) and the clock bias stay double.
// Measurement prediction runs in double once per satellite per epoch.
// Pure and host-compilable (tests_cpp/test_raw_gnss_ekf.cpp). The once-an-
// epoch code -- the carrier's, and the boost screen's gate -- is
// TR_RawGnssCarrier.cpp's, which the Beetle keeps in flash where the rest
// runs from internal RAM (raw_gnss_iram.lf); the geometry both predict with
// is RawGnssEKFGeometry.h's.
#ifndef TR_RAW_GNSS_EKF_H
#define TR_RAW_GNSS_EKF_H

#include <cstdint>

#include "DopplerChange.h"
#include "RawMeas.h"
#include "ReceiverProfile.h"

namespace gnss_raw
{

// The receiver's settings -- its clock model, its Doppler's lag, its carrier
// range rate's -- are its profile's (ReceiverProfile.h, applyReceiverProfile);
// these defaults are the LC86G's.
struct RawGnssEKFParams
{
    // IMU noise model: the flight filter's defaults (TR_GpsInsEKF.h)
    double a_noise        = 0.20;      // m/s^2
    double a_markov_sigma = 0.01;
    double a_markov_tau   = 100.0;     // s
    double w_noise        = 0.00175;   // rad/s
    double w_markov_sigma = 0.00025;
    double w_markov_tau   = 50.0;
    double accel_var      = 0.25;      // pad levelling, (m/s^2)^2
    // Gravity carries no heading: a levelling update (updateAccelLevel), on
    // the ground or in flight, corrects the tilt and the biases and never the
    // rotation about the vertical -- the gain's rows along the vertical, the
    // attitude's and the gyro bias's, are zeroed, Joseph form, so the
    // heading, the gyro bias that turns it, their variances and covariance
    // come through it unchanged and the cross-covariances stay consistent
    // (the dual of att_gate_vertical, which keeps only those rows). Without
    // it, a heading left correlated with the tilt or the accelerometer bias
    // is turned through those correlations, at once or later through that
    // gyro bias. Replayed from the rig's f3c with every epoch from T+0.5 s
    // cut, the first levelling under the chute turned act's 30 deg launch
    // heading -8.8 deg and took its 1-sigma from 31.4 to 13.5 deg, as if
    // gravity had measured it. On a pad with no magnetometer a start a few
    // degrees off its tilt gave the levelling some of the heading's variance
    // as the linearization moved (3 deg off: 57 deg was 41 after 10 s); the
    // heading keeps its own now. With a magnetometer the pad's heading is the
    // magnetometer's either way.
    bool   level_heading_free = true;
    // The magnetometer as a heading (TR_GpsInsEKF's, #1578/#1304)
    double mag_sigma_ut   = 0.5;       // the field's noise per axis, uT
    double mag_tilt_sigma_mps2 = 0.05; // the accelerometer's, as a tilt reference
    double mag_r_floor    = 0.0027;    // the heading variance at least (calibration residuals), rad^2
    double mag_r_ceiling  = 0.06;      // and at most, before the magnitude penalty
    double mag_bias_nis   = 9.0;       // an innovation past this does not move the gyro bias
    // Receiver clock: white frequency (bias) and random-walk frequency
    // (drift) noise. The LC86G steers its clock with its own solution, and
    // its measurements carry drift steps of tens of m/s when that solution
    // breaks (+124 m/s within seconds of a 15 g ignition): 100, not a free
    // TCXO's 0.036.
    double clk_bias_psd   = 0.009;     // m^2/s
    double clk_drift_psd  = 100.0;     // m^2/s^3
    double isb_psd        = 1.0e-4;    // m^2/s: nearly constant
    // Raw measurement gating
    double gate_chi2      = 10.83;     // p = 0.001, 1 dof
    int    reject_persist = 3;         // refused this many epochs running: let back in
    // The Doppler's own count (below 0: reject_persist's). The count is of
    // epochs, and the Beetle's 3 is 3 s at its 1 Hz: a receiver fused at 10 Hz
    // would let a stale or recovering satellite's Doppler back in after 0.3 s,
    // the tail the vertical check's latch was built for. Its pseudorange, fused
    // once a second there (RawGnssShadow's pr_period_s), keeps reject_persist.
    // The boost screen's let-in (screenRangeRates) takes the same count.
    int    reject_persist_rr = -1;
    // A step in every pseudorange (or range rate) at once is the receiver's clock
    double clock_step_m          = 1000.0;
    double clock_step_spread_m   = 200.0;
    double drift_step_mps        = 10.0;
    double drift_step_spread_mps = 2.0;
    // The Doppler is the range rate this long before the epoch's tag (LC86G
    // MSM7, fitted against the rig's truth in boost and coast, 2026-10-12)
    double rr_lag_s       = 0.0555;
    // An epoch whose Dopplers are pairs' means (the mosaic-G5's Dopplers in
    // pairs: setRrPaired) has them this much further back again: the pair's
    // middle, half the receiver's period (the receiver profile's
    // rr_pair_avg). 0: no epoch is (every receiver but that one).
    double rr_pair_lag_s  = 0.0;
    bool   att_gate_cos4  = true;      // false: GNSS corrections reach the attitude in full
    // GNSS corrections to the attitude about the vertical only (wins over
    // att_gate_cos4): the heading, and the gyro biases only along the
    // vertical -- across it they would tilt the attitude later. The tilt
    // stays the gyro's and the levelling's. For a filter that learns its
    // heading in flight (the heading fit's hand-off, RawGnssShadow).
    bool   att_gate_vertical = false;
    // GNSS never corrects the attitude or the gyro bias: the pseudorange and
    // Doppler updates' gain on them masked (Joseph form, so P stays
    // consistent), the levelling, the magnetometer, the barometer and zero
    // velocity untouched. The "unaided" twin a raw filter's attitude is judged
    // against (the replay harness's --unaided): the same firmware, its
    // attitude corrected only by gravity and the magnetometer. Not wholly
    // free of GNSS: those rows still move the accelerometer bias (and its
    // covariance with the attitude), and the levelling takes the
    // accelerometer less that bias as gravity -- so a bias the GNSS taught it,
    // or got wrong, tilts the twin wherever it levels. Small on a still pad;
    // more after a boost and coast that moved the bias. Not the carrier's
    // update: the harness refuses it with the carrier fused.
    bool   att_gnss_off = false;
    // How far a GNSS row may turn the heading, by how well the moment
    // observes it (hdg_obs_mode, HdgObsMode; off by default: today's filter).
    // A heading error turns the specific force's horizontal part f_h, so it
    // grows a velocity error at |f_h| per radian. In a boost that is tens of
    // m/s^2; in the coast only the drag's small sideways part, a tenth of that
    // or less, ties velocity to heading. A velocity error the receiver hands
    // over there (its loops settling after the thrust, a stale satellite, a
    // coherent ramp) is then "fixed" by turning the heading through the
    // correlation the boost left: in Grid B (2026-10-17) 11-44 deg past the
    // unaided twin in the coast, the heading's own 1-sigma reading 8. The
    // rows' gain on the attitude about the vertical, and on the gyro bias
    // along it, is scaled by s in [hdg_obs_floor, 1] -- the tilt's and every
    // other state's untouched, Joseph form so P stays consistent -- with
    //   kHdgObsForce:  s = |f_h| / hdg_obs_fh_mps2, f_h the horizontal
    //                  specific force (NED) averaged over hdg_obs_tau_s (an
    //                  exponential mean of the vector: a coning or rolling
    //                  body's sideways force averages out);
    //   kHdgObsInfo:   s = x^2 / (1 + x^2), x = |f_h| tau sigma_psi /
    //                  hdg_obs_sigma_v: the velocity a 1-sigma heading error
    //                  makes over tau, in the Doppler velocity's noise -- the
    //                  gain a heading-only update on that alone would have, so
    //                  a heading already sure is held harder than one that is
    //                  not (the 30 deg no-magnetometer prior);
    //   kHdgObsWindow: s = 0 for hdg_obs_window_s after the boost (the
    //                  coast's start, where the loops settle), 1 otherwise
    //                  (before any boost too: the pad as today);
    // and s = 1 whenever the IMU read past hdg_obs_open_g within
    // hdg_obs_open_hold_s of the epoch's own time: the boost keeps the full
    // gate. Set once an epoch (beginGnssRaw), so slicing an epoch does not
    // change it. Only the pseudorange and Doppler rows (the carrier's update
    // is not scaled), and nothing with att_gnss_off (already masked).
    // RawGnssShadow's heading_obs_scope says where it applies (everywhere, in
    // flight, or launch to the apogee call: setHdgObsOn).
    uint8_t hdg_obs_mode       = 0;     // HdgObsMode
    double  hdg_obs_fh_mps2    = 2.0;   // kHdgObsForce: open at this |f_h|
    double  hdg_obs_tau_s      = 1.0;   // f_h's averaging time (and kHdgObsInfo's window)
    double  hdg_obs_sigma_v    = 0.1;   // kHdgObsInfo: the Doppler velocity's noise, m/s
    double  hdg_obs_window_s   = 3.0;   // kHdgObsWindow: closed this long after the boost
    double  hdg_obs_open_g     = 2.0;   // the boost: the IMU's specific force past this many g ...
    double  hdg_obs_open_hold_s = 0.2;  //   ... within this long of the epoch: open
    double  hdg_obs_floor      = 0.0;   // s at least this
    // The antenna's offset from the IMU, where the rocket's owner measured it
    // (an optional per-rocket setting; unknown, off, by default: the antenna
    // taken at the IMU, today's filter exactly). While the body turns at
    // omega, an antenna r from the IMU moves C (omega x r) against it -- 0.5
    // m/s at a drogue's 5 rad/s snatch with 0.1 m on the nose -- and sits
    // C r from it. With lever_on every range rate is predicted at the
    // antenna's velocity and every pseudorange at its position, each at its
    // own instant: the attitude and the gyro's rate (less its bias) the
    // filter had then, kept beside the velocity history (lev_). It is a
    // prior, not the truth: what the measurement could be off by is
    // RawGnssShadow's (its antenna_sigma_*, on the Dopplers' sigma and the
    // Doppler change check's lever allowance). The carrier's rate keeps the
    // IMU's point (the mosaic-G5's profile has it off). lever_att_mode: the
    // offset's own information on the attitude -- a turning antenna's
    // velocity turns with the attitude, so it says how the body is rolled
    // about the nose, which nothing else on a vertical thrust does -- unused
    // (off), counted (observe: RawGnssShadow's Doppler change check fits it
    // epoch by epoch; within one epoch it looks just like a velocity error),
    // or fused (fuse: the Doppler rows' terms on the attitude and the gyro
    // bias, d(C (omega x r)), the filter's own velocity telling the two apart
    // as the turn goes on). Not fused, those terms are considered: each
    // Doppler's variance takes what the attitude's own uncertainty does to
    // the antenna's velocity (leverRows).
    bool    lever_on           = false;
    double  lever_b_m[3]       = {0.0, 0.0, 0.0};   // body FRD, the IMU to the antenna's phase centre, m
    uint8_t lever_att_mode     = 0;                 // LeverAttMode
    // Initial sigmas, in the state's units (the attitude's: a half-angle).
    // p0_att: the tilt's, about north and east; p0_hdg: the heading's, about
    // the vertical of init's attitude (#1626 N10). p0_hdg was pi ("any
    // heading", uNavINS's), on body z. About a tilted vertical, pi^2 beside
    // what the levelling pins (the tilt with the accelerometer bias, to 4e-6)
    // is past float's precision: on a still pad with no magnetometer the
    // attitude block went indefinite within two minutes up the rail and the
    // heading's sigma restarted at 0.2 deg. 0.5 (a heading 1-sigma of
    // 57 deg) held 30 minutes at every rail attitude tried, 75-90 deg
    // (UpTheRailTheLevelledStartKeepsItsHeadingFreeInFloat). At 10-60 deg a
    // still pad gives float out after 4-7 minutes with either start: there the
    // levelled tilt itself nears float's floor. The flight filter takes 30 deg
    // about the vertical on the pad (PAD_HEADING_SIGMA_DEG).
    double p0_pos = 10.0, p0_vel = 1.0, p0_att = 0.34906, p0_hdg = 0.5;
    double p0_abias = 0.981, p0_wbias = 0.01745;
    double p0_clk_bias = 30.0, p0_clk_drift = 5.0, p0_isb = 30.0;
    // Gravity: the flight filter's constant 9.807, or WGS84 normal gravity
    bool   gravity_wgs84  = false;
    // The velocity-increment history keeps an entry per this much time, s
    // (128 entries: 0.5 s at 4 ms; at a 200 Hz IMU, every sample)
    double hist_spacing_s = 0.004;
    // The carrier range rate (ReceiverProfile.h's cr_*)
    uint8_t cr_mode             = kCarrierObserve;   // CarrierMode
    double  cr_sigma_per_rev_s  = 0.023;             // m/s more (in quadrature) per rev/s of roll
    double  cr_common_sigma_mps = 5.0;               // the epoch's common rate: its prior 1-sigma
    double  cr_screen_k         = 4.0;               // the screen: suspect past this many sigmas
    double  cr_screen_floor_mps = 0.03;              //   and this much
    uint8_t cr_half_cycle       = kHalfCycleReject;  // HalfCycleAction
    // each system's carrier (its wavelength), and whether its half-cycle
    // steps can come unflagged (gnss_sys_t: GPS, Galileo, BeiDou, QZSS)
    double  cr_carrier_hz[kProfileSys] = {GNSS_FREQ_L1_HZ, GNSS_FREQ_L1_HZ, 1561.098e6, GNSS_FREQ_L1_HZ};
    bool    cr_half_cycle_unflagged[kProfileSys] = {true, false, true, true};
};

// RawGnssEKFParams::hdg_obs_mode
enum HdgObsMode : uint8_t
{
    kHdgObsOff = 0,      // the full gate: today's
    kHdgObsForce = 1,    // by the horizontal specific force
    kHdgObsInfo = 2,     // by the heading's information from it, against its own
    kHdgObsWindow = 3,   // none in the coast's first seconds
};

// RawGnssEKFParams::lever_att_mode
enum LeverAttMode : uint8_t
{
    kLeverAttOff = 0,      // the offset predicted, its attitude terms unused
    kLeverAttObserve = 1,  // and the roll they point to counted (RawGnssShadow's change check), nothing fused
    kLeverAttFuse = 2,     // and fused: the Doppler rows' attitude and gyro-bias terms
};

struct RawGnssStats
{
    int    used_pr = 0, used_rr = 0;
    int    rejected_pr = 0, rejected_rr = 0;
    int    clock_jump_ms = 0;          // a whole-millisecond clock step taken
    double clock_step_m = 0.0;         // a common pseudorange step that was not
    double drift_step_mps = 0.0;       // a common range-rate step
    float  nis_max = 0.0f;             // largest NIS among the updates applied
    // the carrier range rates (beginCarrier, fuseCarrier)
    int    used_cr = 0;                // fused
    int    cr_built = 0;               // offered (has_cr)
    int    cr_unmatched = 0;           //   whose earlier epoch the filter did not keep (or no clock yet)
    int    cr_in = 0;                  //   screened in (and repaired): fused in fuse mode
    int    cr_half = 0;                //   rejected as half-cycle steps
    int    cr_repaired = 0;            //   repaired, half-cycle steps
    int    cr_other = 0;               //   rejected otherwise
    bool   cr_too_few = false;         // under 4 to screen: none taken
    float  cr_rms_mps = 0.0f;          // the screened rates' innovations about the common rate, RMS
    float  cr_fit_rms_mps = 0.0f;      //   and about the screen's fit (the common rate and a velocity)
    float  cr_dv_mps = 0.0f;           // that fit's velocity correction: the prior's error, as the rates see it
    float  cr_roll_rev_s = 0.0f;       // the mean |roll rate| over the interval, rev/s
    double cr_common_mps = 0.0;        // the epoch's common rate: observe, the screen's fit to the screened
                                       //   rates (a weighted mean, the velocity's part taken out); fuse,
                                       //   the estimate
    // RawGnssShadow's boost screen: the Dopplers gated at the epoch's prior
    // before any update (screenRangeRates), and the epoch worked on a scratch
    // copy of the filter (its updates' own counts, the Dopplers')
    int    screen_rr = 0;              // gated at the prior
    int    screen_refused = 0;         //   of them, refused
    int    screen_seq_used = 0;        // the scratch copy's: the Dopplers fused
    int    screen_seq_refused = 0;     //   and refused
    bool   screen_dropped = false;     // the epoch dropped, none of it fused (the shadow's call)
    // and the vertical check's fit (screenRangeRates): one velocity and clock
    // rate fitted to the epoch's Dopplers at the same prediction
    int    screen_fit_n = 0;           // the Dopplers fitted (0: under 4, or no fit)
    int    screen_fit_out = 0;         //   left out of it, each far off on its own
    float  screen_dv_down_mps = 0.0f;  //   the fit's velocity correction, down: the epoch's vertical
                                       //   velocity against the filter's
};

// One carrier range rate as the screen took it (RawGnssEKF::carrierInfo).
struct CarrierInfo
{
    enum : int8_t { kIn = 0, kRepaired = 1, kHalfCycle = 2, kOther = 3, kUnmatched = 4, kTooFew = 5 };
    uint8_t idx;        // its index in the epoch's measurements
    uint8_t sys, prn;
    int8_t  verdict;    // above
    int8_t  half;       // a half-cycle step found: +1 (lambda/2 too much), -1, 0 none
    float   sigma;      // its 1-sigma, m/s (with the roll rate's term)
    double  y;          // its innovation at the prior before the common rate, m/s (a repair taken out)
};

// One epoch's Doppler range rates as the next epoch's change is taken from
// (RawGnssEKF::dopplerChange, RawGnssShadow's dv_check)
struct DopplerRef
{
    bool    valid = false;
    double  t = 0.0;                         // the Dopplers' instant (the epoch's less its lag, rrLag), filter time
    float   v_ned[3] = {};                   // the filter's velocity then, as it stood when taken: the base
                                             //   both ends predict from (it cancels)
    int     n = 0;
    uint8_t sys[RawEpoch::kMaxObs] = {};
    uint8_t prn[RawEpoch::kMaxObs] = {};
    double  y[RawEpoch::kMaxObs] = {};       // each Doppler less its prediction at v_ned (the clock's drift in it)
    float   s[RawEpoch::kMaxObs] = {};       // its 1-sigma, m/s
    float   u[RawEpoch::kMaxObs][3] = {};    // its line of sight, NED, receiver to satellite (the latch's level
                                             //   fit, RawGnssShadow's dv_latch_k)
};

// What the IMU did over a Doppler change's interval (RawGnssEKF::dopplerChange)
struct DopplerChangeImu
{
    bool   ok = false;     // the history covers the interval (else no rows)
    double dt = 0.0;       // the interval, s
    float  dv[3] = {};     // the IMU's velocity change over it, NED (specific force and gravity, no corrections)
    float  dv_sf[3] = {};  //   the specific force's part
    float  v_lev[3] = {};  // lever_on: the antenna's velocity against the IMU at this end's Dopplers, NED, as
                           //   every prediction here took it (0 with the lever off; set with or without a ref)
};

namespace ekf_geom
{
struct Los;   // a satellite's predicted range, rate and line of sight (RawGnssEKFGeometry.h)
}

template <typename T>
class RawGnssEKF
{
public:
    static constexpr int N = 19;
    enum : int
    {
        kPos = 0, kVel = 3, kAtt = 6, kAccBias = 9, kGyroBias = 12,
        kClkBias = 15, kClkDrift = 16, kIsbGal = 17, kIsbBds = 18
    };
    static constexpr int kHist = 128;

    explicit RawGnssEKF(const RawGnssEKFParams& p = RawGnssEKFParams());
    void setParams(const RawGnssEKFParams& p);
    const RawGnssEKFParams& params() const { return p_; }
    // att_gate_vertical, switched in flight (the heading fit's act mode: from
    // launch -- on the pad nothing sees the heading for GNSS to turn)
    void setAttGateVertical(bool on) { p_.att_gate_vertical = on; }
    // att_gate_cos4, switched in flight (act's full gate: GNSS corrections
    // reach the whole attitude, att_gate_vertical off as well)
    void setAttGateCos4(bool on) { p_.att_gate_cos4 = on; }
    // hdg_obs_mode applied (true, the default) or not (s = 1): where it
    // applies is the shadow's (heading_obs_scope)
    void setHdgObsOn(bool on) { hdg_obs_on_ = on; }
    // The antenna's offset (lever_on, lever_b_m, lever_att_mode), set on the
    // ground where the owner's setting changes: the history's earlier
    // entries keep the offset they were taken with (on the ground the body
    // is still, so its velocity term was 0 anyway).
    void setLever(bool on, const double b_m[3], uint8_t att_mode);
    // s for an epoch age_s old at the state as it stands (hdg_obs_mode), and
    // the one the last epoch begun took (1 with the mode off)
    T hdgObsScaleNow(double age_s) const;
    T hdgObsScale() const { return hdg_obs_s_; }
    // the horizontal specific force as kHdgObsForce and kHdgObsInfo average
    // it (NED north, east; m/s^2), and the filter time the IMU last read past
    // hdg_obs_open_g (-1e9: never since init) -- tracked only with the mode on
    const T* hdgObsForce() const { return fh_ew_; }
    double hdgObsOpenT() const { return hdg_open_t_; }

    // Start at a position (lat, lon rad; ellipsoidal height m), NED velocity
    // and attitude, at filter time t0 (s; any clock the IMU steps share).
    // The clock is set by the first raw epoch (initClockFrom). The attitude's
    // covariance: p0_att about the two horizontal axes, p0_hdg about the
    // vertical of q (#1626 N10) -- so q is the best attitude there is (on the
    // pad, the accelerometer's tilt), not a placeholder.
    void init(const double lla[3], const T v_ned[3], const T q[4], double t0);
    // Take the attitude as known (the pad seed): its variance to 1e-6 and its
    // cross-covariances to zero, as GpsInsEKF::setQuaternion.
    void setQuaternion(const T q[4]);
    // The gyro bias from a still gyro's mean (the pad seed).
    void seedGyroBias(const T gyro_mean[3]);
    // The heading's own 1-sigma (rad, a rotation about the vertical), as a
    // magnetometer or the heading fit hands it over: the attitude's variance
    // about the vertical set to it, its cross-covariances dropped, the tilt's
    // kept. (setQuaternion takes the whole attitude as known: right for the
    // tilt the pad levels, not for a heading a filter must learn in flight.)
    void setHeadingSigma(T sigma_rad);
    // The tilt's 1-sigma about north and about east (rad, a rotation) at
    // least sigma_rad: each one's deficit added along it, so the part about
    // the vertical, the north-east covariance and every cross-covariance are
    // kept, P symmetric and positive definite. (An axis between north and
    // east can stay under the floor by that covariance: little, once the
    // floor is past the tilt the pad left.) For a tilt the pad could not tell
    // from an accelerometer bias (#1626 K1), let go at launch (RawGnssShadow's
    // heading_launch_tilt_sigma_deg).
    void setTiltSigmaFloor(T sigma_rad);
    // Turn the attitude about the vertical (NED down) by dpsi (rad): the
    // heading grows by dpsi, the tilt is kept.
    void turnHeading(T dpsi);
    // The IMU biases and their 6x6 covariance (accelerometer then gyro), in
    // place of the filter's own, their cross-covariances dropped: a heading
    // hand-off puts back what the pad left, before a wrong heading leaned on
    // them in flight.
    void setBiases(const T ab[3], const T wb[3], const T Pb[6][6]);
    void biasCovariance(T Pb[6][6]) const;
    // Let the velocity go: each axis's variance at least sigma^2 (m/s), its
    // cross-covariances dropped, its value kept.
    void releaseVelocity(T sigma);
    // Forget which satellites were refused (reject_persist's count): ones
    // refused under a wrong heading come back on their own merits.
    void clearRejects();
    // Whether updateGnssRawPart fuses the pseudoranges (the default) or only
    // the Dopplers. A receiver faster than 1 Hz has its code fused once a
    // second (RawGnssShadow's pr_period_s): each code update's correlated part
    // is inflated by the time since that satellite's last (corrInflation), so
    // fused ten times a second the gate opens about sqrt(10) wider for the
    // same information. Off, an epoch's pseudoranges still start and step the
    // clock (initClockFrom, beginGnssRaw), and each satellite's code refusals
    // and last update stand as they were. A copy of the filter takes it along.
    void setFusePr(bool on) { fuse_pr_ = on; }
    bool fusePr() const { return fuse_pr_; }
    // GNSS held off the attitude and the gyro bias, for the updates until it
    // is let go: what att_gnss_off does always, here for a while (the
    // pseudorange and Doppler rows' gain on them masked, Joseph form; the
    // carrier's update is not held). RawGnssShadow's hold while the body
    // turns fast (Config::att_hold_rate_rps): an antenna off the IMU swings
    // then, and its Dopplers would turn the attitude. A copy of the filter
    // takes it along; init lets it go.
    void setAttHold(bool on) { att_hold_ = on; }
    bool attHold() const { return att_hold_; }
    // The epoch about to be worked has its Dopplers in pairs (rr_pair_lag_s;
    // RawGnssShadow sets it at each epoch's start where the params have a
    // pair lag, from the epoch's BuiltEpoch::rr_paired): its Dopplers are
    // taken rr_pair_lag_s further back by every consumer -- the update, the
    // boost screen and vertical check, the change check -- and each
    // satellite's own range acceleration is taken once for them all
    // (pairAccel). An epoch not in pairs takes rr_lag_s alone, as every epoch
    // did before; with no pair lag in the params none is. A copy of the
    // filter takes it along.
    void setRrPaired(bool on)
    {
        rr_paired_ = on && p_.rr_pair_lag_s > 0.0;
        acc_memo_.have = 0;
    }
    bool rrPaired() const { return rr_paired_; }
    // The epoch's Dopplers' lag (s, at least 0): rr_lag_s, plus rr_pair_lag_s
    // on an epoch in pairs. rrPairLag() is the second part alone (0 off one).
    double rrLag() const
    {
        const double tau = rr_paired_ ? p_.rr_lag_s + p_.rr_pair_lag_s : p_.rr_lag_s;
        return tau > 0.0 ? tau : 0.0;
    }
    double rrPairLag() const { return rr_paired_ ? p_.rr_pair_lag_s : 0.0; }
    // A flight restored after a reboot: the position's, velocity's and
    // attitude's 1-sigmas set to these on every axis (the attitude's a
    // rotation, rad), their cross-covariances dropped. A block whose sigma is
    // not above 0 is left as it was.
    void inflateRestoreSigmas(T pos_m, T vel_mps, T att_rad);
    // The accelerometer bias (m/s^2, body), its covariance kept: a restored
    // flight's, the snapshot's.
    void setAccelBias(const T ab[3]);

    // IMU time update: specific force (m/s^2) and rate (rad/s), body FRD.
    void propagate(const T acc[3], const T gyro[3], T dt);
    // Gravity-reference update on the pad (GpsInsEKF::accelMeasUpdate), and
    // in flight where the shadow levels. With level_heading_free (the
    // default) it never turns the heading or shrinks its variance.
    void updateAccelLevel(const T acc[3]);
    // A height above the ellipsoid (m) measured with variance var (m^2): the
    // barometer, referred to the filter's own pad. No innovation gate: a
    // gate refuses the very barometer that would bring a drifted altitude
    // back (GpsInsEKF::baroMeasUpdate, #1579). It moves the vertical position
    // and velocity and the receiver clock only: a height says nothing about
    // the attitude, the body's biases or the horizontal.
    void updateHeight(double h_m, T var);
    // Zero velocity, sigma (m/s) per axis, into position and velocity only:
    // a velocity measurement cannot tell an attitude error from an
    // accelerometer bias (GpsInsEKF::zeroVelocityUpdate, #1418).
    void updateZeroVelocity(T sigma);
    // The magnetometer (body FRD, uT) as a heading, at any attitude: its
    // field carried into NED on the state's attitude turned onto the tilt
    // reference by the shortest arc -- the accelerometer's down when it
    // measures gravity (accel_is_gravity: the pad, the ground), else the
    // state's own -- and that azimuth set against magnetic north
    // (declination_rad, true north east of magnetic). field_err_ut: |B| less
    // the model's, a hard iron that size can turn the heading (#1304); < 0
    // unknown. False if the field's horizontal part is too weak to say
    // (TR_GpsInsEKF::magMeasUpdate, #1578: on a vertical rail too).
    bool updateMagHeading(const T acc[3], const T mag[3], bool accel_is_gravity, T declination_rad,
                          T field_err_ut);

    // Clock bias and drift from an epoch's medians at the current position;
    // false with no pseudorange. updateGnssRaw calls it until it succeeds.
    bool initClockFrom(const RawMeas* m, int n, double age_s = 0.0);
    // Every satellite's pseudorange and range rate of one epoch, which was
    // age_s ago.
    RawGnssStats updateGnssRaw(const RawMeas* m, int n, double age_s = 0.0);
    // The same in parts, for a filter that must keep stepping while it fuses
    // a sky: beginGnssRaw takes the epoch's receiver clock (its first value, a
    // step every satellite agrees on; false with no clock yet), then
    // updateGnssRawPart the satellites [i0, i1), each part at the age the
    // epoch has when it is fused. updateGnssRaw is the two at one age.
    bool beginGnssRaw(const RawMeas* m, int n, double age_s, RawGnssStats& st);
    void updateGnssRawPart(const RawMeas* m, int i0, int i1, double age_s, RawGnssStats& st);
    // The epoch's Doppler range rates (has_rr) gated at the state as it
    // stands, after beginGnssRaw and before any update: each with the
    // innovation, variance and chi-square updateGnssRawPart would give it as
    // the epoch's first update (a satellite refused reject_persist epochs
    // running -- reject_persist_rr where set -- passes: the update lets it
    // in). How many were gated and
    // refused, in st.screen_rr and st.screen_refused; nothing else changes.
    void screenRangeRates(const RawMeas* m, int n, double age_s, RawGnssStats& st) const;
    // the vertical check's fit leaves out a Doppler this many of its own 1-sigmas off the rest's fit
    static constexpr double kVertOutlier = 4.0;
    static bool fitVelocityClock(const double H[][4], const double* y, const double* s, const bool* in, int n,
                                 double x[4]);
    static bool verticalFit(const double H[][4], const double* y, const double* s, bool* in, int n, double x[4],
                            int& nin, int& nout);
    // The Doppler change check's rows (DopplerChange.h; RawGnssShadow's
    // dv_check), an epoch age_s ago, before any of it is fused: each Doppler
    // (has_rr) whose satellite ref also has, its change since ref less the
    // change predicted at ref's velocity carried by the IMU's own velocity
    // change since (imuMotion: no filter correction in it) -- the satellite's
    // own motion from its ephemeris, the clock's drift left to the fit. Its s:
    // its sigma_rr and ref's, in quadrature. out is this epoch's reference for
    // the next (at the filter's velocity as it stands). rows (up to kMaxObs)
    // and how many; 0 with no ref, or if the IMU's history does not cover the
    // interval (imu.ok false). Nothing in the filter changes.
    int  dopplerChange(const RawMeas* m, int n, double age_s, const DopplerRef& ref, DopplerRef& out,
                       dvc::Row* rows, DopplerChangeImu& imu) const;
    // The epoch begun is dropped, none of it fused: its receiver is no
    // carrier rate's earlier end, so the next epoch's rates across it go
    // unmatched. endCarrier still closes it.
    void dropEpoch() { cr_cur_.valid = false; }
    // The epoch's carrier range rates (above), after its pseudoranges and
    // Dopplers, the epoch age_s ago: beginCarrier screens them at the prior
    // and counts (observe mode: all there is; it returns 0), and returns how
    // many fuseCarrier will fuse (fuse mode). fuseCarrier fuses every one
    // with the epoch's common rate, at once: nothing may step or update the
    // filter between its rates (the common rate's cross-covariance is the
    // state's at that instant). endCarrier closes the epoch -- the common
    // rate dropped, this epoch's receiver kept for the next -- whether or not
    // the others ran: call it once an epoch. updateGnssRaw does all three.
    int  beginCarrier(const RawMeas* m, int n, double age_s, RawGnssStats& st);
    void fuseCarrier(const RawMeas* m, RawGnssStats& st);
    void endCarrier(RawGnssStats& st);
    // The last screen's rates, n of them.
    const CarrierInfo* carrierInfo(int& n) const { n = cr_ninfo_; return cr_info_; }

    double time() const { return t_; }
    const double* lla() const { return lla_; }
    const T* velNed() const { return v_; }
    const T* quat() const { return q_; }
    const T* accelBias() const { return ab_; }
    const T* gyroBias() const { return wb_; }
    double clockBias() const { return clk_b_; }
    T clockDrift() const { return clk_d_; }
    T isb(int sys) const;                       // gnss_sys_t; GPS is the reference: 0
    bool clockReady() const { return clk_ready_; }
    T P(int i, int j) const { return P_[i][j]; }
    T sigma(int i) const;
    // The attitude's 1-sigma (rad) as rotations about north, east and down
    // (down: the heading): the attitude block, a body-frame half-angle,
    // doubled and carried into NED.
    void attSigmaNed(T out[3]) const;
    // |specific force| / g - 1 at the last IMU step, and its largest since
    // the last take (floored at 0): what RawMeasBuilder::build takes.
    T gExcess() const { return g_excess_; }
    T takeGPeak();
    // The body's rate (the gyro less its bias, rad/s) at its largest since
    // the last take: its whole size, and its size across the nose (body y
    // and z: the rate that swings an antenna on the nose's axis), each
    // largest on its own; both zeroed by the take. Kept only while
    // setTrackRate is on (RawGnssShadow's rotation allowance and hold), so a
    // filter without them steps as it always has; off, both read 0.
    void setTrackRate(bool on) { track_rate_ = on; }
    void takeRatePeak(T& all, T& across);
    // The receiver in ECEF (m, m/s).
    void receiverEcef(double r[3], double v[3]) const;
    // The receiver age_s ago (s), ECEF: now taken back by the history, as an
    // epoch that age is applied (#1508's satellite check: before an update,
    // the IMU's propagation from the last one).
    void receiverEcefAt(double age_s, double r[3], double v[3]) const;
    // What the IMU alone did between filter times t1 <= t2 (s), NED: its
    // velocity change (specific force and gravity, no filter corrections) and
    // the displacement that change adds, the integral of v(t) - v(t1). From a
    // state at t1, the IMU predicts p(t2) = p(t1) + v(t1) (t2 - t1) + dp.
    // False, with both zero, unless the history covers [t1, t2].
    bool imuMotion(double t1, double t2, T dv[3], T dp[3]) const;
    // The antenna against the IMU age_s ago (lever_on): its offset C r (m)
    // and velocity C (omega x r) (m/s), NED, interpolated between the
    // history's entries (the oldest's before them, the newest's after), and
    // where asked the nose (body x) then, NED, and the attitude then (body ->
    // NED, scalar first, as quat(); between two entries their normalized
    // mean, the nose its first column). All zero with lever_on off; with no
    // history yet, at the attitude now and no velocity.
    void leverAt(double age_s, T p_ned[3], T v_ned[3], T nose_ned[3] = nullptr, T q_ned[4] = nullptr) const;

private:
    // The antenna at an epoch (lever_on): its offset at the epoch (age_s
    // ago) and its velocity there, and at the Doppler's instant (tau before
    // it) its offset, its velocity and the attitude (body -> NED), NED --
    // what leverBack takes off the history's change so the receiver the
    // predictions back-date is the antenna, not the IMU, and what leverRows
    // takes for the gyro bias's term at the Doppler's instant.
    struct LeverEpoch
    {
        T p_e[3], v_e[3], p_l[3], v_l[3];
        T C_l[3][3];
    };
    void   leverBack(double age_s, double tau, T dp_e[3], T dv_e[3], T dv_l[3], LeverEpoch& lv) const;
    // propagate's: the antenna's offset and velocity and the attitude at the
    // step's end, into the history entry the step went into (lev_)
    void   pushLever(const T w_b[3]);
    // A Doppler row's lever terms (lever_on): the attitude's, 2 (u x v)' C,
    // and the gyro bias's, -(u x p)' C_l (u the line of sight, v and p the
    // antenna's velocity and offset at the Doppler's instant, NED; C body ->
    // NED now, the attitude state's frame, and C_l then: a bias turns the
    // rate the body had then, in the body as it stood then). Fuse
    // (lever_att_mode): appended to the row's H at hidx/hval (hn so far);
    // returns the new count. Otherwise they are considered, not fused: R
    // takes what the attitude's and gyro bias's own uncertainty does to the
    // antenna's velocity, h P h' -- the predicted velocity is turned by any
    // attitude error, and a heading 10 deg off turns a 1.5 m/s swing by
    // 0.26 m/s -- and hn is returned as it was.
    int    leverRows(const T u_ned[3], const LeverEpoch& lv, int* hidx, T* hval, int hn, T& R) const;
    struct Hist
    {
        double t;      // end of the entry's span, filter time
        T      dv[3];  // NED velocity increment over the span
        T      dp[3];  // NED displacement over it
        T      ex[3];  // the carrier's: how far that displacement (each step's end velocity times
                       // its length) runs past the velocity's own path, sum of a dt^2 / 2
    };

    void   gravityNed(T g[3]) const;
    // f_b: the specific force less the accelerometer bias, body (the tilt's
    // Jacobian is its, not the kinematic acceleration's: #1626 E1)
    void   propagateCovariance(const T Tb2n[3][3], const T f_b[3], const T w_b[3], T g_d, T dt);
    void   stabilize();
    void   symmetrize();
    T      hph(const int* hidx, const T* hval, int hn) const;
    // kmask: the states the gain may move (bit i, state i); the rest keep
    // their estimate and, Joseph form, a consistent covariance. att_axis: the
    // gain's attitude and gyro-bias rows projected onto this body-frame unit
    // vector (att_gate_vertical: the vertical). att_off_axis: the gain's
    // attitude and gyro-bias rows along this body-frame unit vector zeroed
    // (the levelling, the vertical: the attitude about it and the gyro bias
    // along it keep their estimates and covariance). off_keep: the share of
    // those rows along att_off_axis kept instead (0, the levelling's: none;
    // the GNSS rows' hdg_obs_mode scale).
    int    scalarUpdate(const int* hidx, const T* hval, int hn, T y, T R, T gate, T att,
                        T dx[N], T* nis, uint32_t kmask = 0xFFFFFFFFu, const T* att_axis = nullptr,
                        const T* att_off_axis = nullptr, T off_keep = T(0));
    // The gain's attitude rows as the GNSS updates have them: scaled by att,
    // and (att_axis) the attitude's and gyro bias's projected onto that axis
    // (gnssAttAxis: the vertical in the body frame with att_gate_vertical,
    // else none).
    static void gateGain(T K[N], T att, const T* att_axis);
    const T* gnssAttAxis(T vert[3]) const;
    // reject_persist for a kind (0 pseudorange, 1 range rate): the Doppler's
    // own count where it has one (reject_persist_rr)
    int    rejectPersist(int kind) const
    {
        return kind == 1 && p_.reject_persist_rr >= 0 ? p_.reject_persist_rr : p_.reject_persist;
    }
    // One carrier rate with the epoch's common rate c: x and c jointly (its
    // H 1; P_xc, P_cc beside P), Joseph form, the gain gated as above. dc:
    // c's correction.
    int    carrierUpdate(const int* hidx, const T* hval, int hn, T y, T R, T att, const T* att_axis, T dx[N],
                         T& dc, T* nis);
    // The carrier's rate at the state as it stands, before the common rate
    // (m/s), and its line of sight at the epoch (NED, the present frame Tm).
    double carrierRate(const RawMeas& m, const double Tm[3][3], T u_ned[3]) const;
    void   crEnds(const double Tm[3][3], double rk[3], double rp[3]) const;
    // this epoch's receiver snapshot, age_s back (dp_e: the history's displacement since)
    void   crSnapshot(double age_s, const T dp_e[3]);
    // The screen's fit (beginCarrier): the epoch's rates, their lines of
    // sight, the prior's; per rate, in the fit, its residual left out of the
    // fit and that residual's sigma. A member, not the stack's:
    // 2.6 kB, in the filter's own storage (PSRAM on the flight computer).
    // (Scratch, each epoch's: written before it is read, so not zeroed.)
    struct CrFit
    {
        int    n = 0;
        int    info[RawEpoch::kMaxObs];     // its cr_info_ entry
        double y[RawEpoch::kMaxObs];        // its innovation before the common rate (a repair taken out)
        double u[RawEpoch::kMaxObs][3];     // its line of sight, NED
        bool   in[RawEpoch::kMaxObs];
        double d[RawEpoch::kMaxObs];
        double sd[RawEpoch::kMaxObs];
        double pvv[3][3];                   // the filter's velocity covariance
        double c0 = 0.0, sc2 = 0.0;         // the common rate's prior: its mean and variance
    };
    bool   crFitSolve(double z[4]);
    void   inject(const T dx[N]);
    T      attGate() const;
    void   pushHistory(T dt, const T a_ned[3]);
    bool   historyChange(double tau, T dv[3], T dp[3]) const;
    // The carrier's: the history's excess (ex) over the last tau.
    void   historyExcess(double tau, double ex[3]) const;
    void   receiverState(double r[3], double v[3], double Tm[3][3]) const;
    // An epoch in pairs (setRrPaired): m's own range acceleration
    // (satRangeAccel at L and the receiver's velocity v, ECEF), m being the
    // epoch's measurement i -- taken once a satellite an epoch (acc_memo_).
    // In TR_RawGnssCarrier.cpp (flash on the Beetle, which never pairs: its
    // per-step code keeps only the call).
    double pairAccel(int i, const RawMeas& m, const ekf_geom::Los& L, const double v[3]) const;

    RawGnssEKFParams p_;
    // process noise, from p_
    T      q_acc_, q_att_, q_ab_, q_wb_, f_ab_, f_wb_, q_clkb_, q_clkd_, q_isb_;

    double lla_[3] = {0.0, 0.0, 0.0};
    T      v_[3] = {};
    T      q_[4] = {1, 0, 0, 0};
    T      ab_[3] = {};
    T      wb_[3] = {};
    double clk_b_ = 0.0;
    T      clk_d_ = 0;
    T      isb_[2] = {};
    T      P_[N][N] = {};
    double t_ = 0.0;
    bool   clk_ready_ = false;
    bool   fuse_pr_ = true;      // setFusePr
    bool   att_hold_ = false;    // setAttHold
    bool   track_rate_ = false;  // setTrackRate
    T      a_ned_[3] = {};
    T      g_excess_ = 0;
    T      g_peak_ = 0;
    // hdg_obs_mode: the horizontal specific force's mean, NED (m/s^2); when
    // the IMU last read past hdg_obs_open_g; applied or not; the last epoch's s
    T      fh_ew_[2] = {};
    double hdg_open_t_ = -1.0e9;
    bool   hdg_obs_on_ = true;
    T      hdg_obs_s_ = T(1);
    T      rate_peak_ = 0;       // takeRatePeak: the whole rate's largest,
    T      rate_across_peak_ = 0;   //   and across the nose
    // screenRangeRates' vertical-check fit: its rows, scratch (kept off the task's stack)
    mutable double vfit_h_[RawEpoch::kMaxObs][4] = {};
    mutable double vfit_y_[RawEpoch::kMaxObs] = {};
    mutable double vfit_s_[RawEpoch::kMaxObs] = {};
    mutable bool   vfit_in_[RawEpoch::kMaxObs] = {};

    Hist   hist_[kHist] = {};
    // lever_on: each history entry's antenna offset C r (0-2) and velocity
    // C (omega x r) (3-5), NED, and the attitude q (6-9, body -> NED, scalar
    // first), at the entry's end (beside hist_, slot for slot; written only
    // with the lever on: an entry from before it has q zero)
    T      lev_[kHist][10] = {};
    int    hhead_ = 0;        // next slot
    int    hn_ = 0;
    double hopen_ = 0.0;      // start of the newest entry's span

    // per satellite (gnss_sys_t x PRN) and kind (0 pseudorange, 1 range rate)
    uint8_t rejects_[3][64][2] = {};
    double  last_upd_[3][64][2] = {};

    // The carrier range rate (above). A receiver snapshot: where the filter
    // put the receiver at an epoch (ECEF), and the corrections since, NED, as
    // they move the trajectory there (inject).
    struct CrSnap
    {
        bool   valid = false;
        double t = 0.0;            // the epoch's filter time; the rest written when it is taken
        double r[3];
        double d_ned[3];
        double ex[3];              // the integration's excess to then (cr_ex_ back-dated), NED
    };
    CrSnap      cr_cur_;           // this epoch's (beginGnssRaw)
    CrSnap      cr_prev_;          // the epoch before's (endCarrier)
    CarrierInfo cr_info_[RawEpoch::kMaxObs];   // the last screen's, cr_ninfo_ of them (not zeroed)
    int         cr_ninfo_ = 0;
    int         cr_nfuse_ = 0;     // of them, to fuse (fuse mode)
    double      cr_c_ = 0.0;       // the epoch's common rate (m/s), its variance, its
    T           cr_pcc_ = 0;       //   cross-covariance with the state (set when fused)
    T           cr_pxc_[N];
    double      cr_roll_sum_ = 0.0, cr_roll_t_ = 0.0;   // |roll rate| dt, and dt, since the last screen
    // The position is integrated on each step's end velocity: over a second
    // it runs a dt / 2 per second past the path the velocity itself traces
    // (6 cm at 13 m/s^2 and 10 ms steps), which the carrier would read as a
    // velocity error. Its running sum, NED (sum of a dt^2 / 2): the carrier's
    // displacement is the position's less this one's change.
    double      cr_ex_[3] = {};
    CrFit       cr_fit_;

    // The Dopplers in pairs (setRrPaired), last, so no member before moves:
    // this epoch's, and each satellite's own range acceleration as the first
    // of its Doppler's consumers took it (pairAccel), by its index in the
    // epoch's measurements (bit i of have: m[i], checked by its satellite;
    // the rest not zeroed, read only under their bit), emptied by setRrPaired
    // at each epoch's start
    bool rr_paired_ = false;
    struct AccMemo
    {
        uint64_t have = 0;
        uint8_t  sys[RawEpoch::kMaxObs];
        uint8_t  prn[RawEpoch::kMaxObs];
        double   a[RawEpoch::kMaxObs];
    };
    static_assert(RawEpoch::kMaxObs <= 64, "AccMemo::have is 64 bits");
    mutable AccMemo acc_memo_;
};

extern template class RawGnssEKF<float>;
extern template class RawGnssEKF<double>;

// The range from r (ECEF, m) to m's satellite, turned by the Earth's rotation
// over the signal's travel: what the pseudorange update predicts, less the
// receiver's clock (the pad survey's residuals, PadSurvey.h).
double sagnacRange(const RawMeas& m, const double r[3]);
// The range rate a receiver still at r (ECEF, m) would see from m's satellite
// -- its Sagnac-turned velocity along the line of sight, m/s -- and that line
// of sight (unit, ECEF, receiver to satellite), in double: what a Doppler
// velocity solve predicts (HeadingFit.h's own, in float, is held to it), and
// the synthetic skies' Doppler in its tests and bench.
double stillRangeRate(const RawMeas& m, const double r[3], double u[3]);

// TR_RawGnssEKF.cpp's own, for the carrier's source (TR_RawGnssCarrier.cpp):
// the Sagnac-turned range from r to a satellite at rs (transmit time, ECEF
// then) and its line of sight, as every range the filter predicts; numpy's
// median, in place.
namespace ekf_detail
{
double sagnacLos(const double rs[3], const double r[3], double u[3]);
double medianOf(double* v, int n);
}  // namespace ekf_detail

}  // namespace gnss_raw

#endif  // TR_RAW_GNSS_EKF_H
