// RawGnssShadow.h — the raw-GNSS shadow filter (#1600 phase 4): TR_RawGnssEKF
// run beside the flight filter, on the flight filter's own IMU input and the
// LC86G's raw epochs, logged (RAW_GNSS_EKF_MSG) and never acted on.
//
// Start and the pad. It starts on the pad from the flight filter's state
// (ShadowSeed). While the rocket is still on the pad it keeps taking the
// flight filter's attitude and gyro bias, once a resync_period_s: the
// heading there is the magnetometer's, and this filter has no magnetometer
// (the bench prototype took the true heading). Until its own first raw
// epoch it takes the position and velocity too. From that epoch on, its
// position, velocity and receiver clock are its own; from launch, all of it is.
//
// Raw epochs. Each comes with its GPS instant on the MCU clock
// (TimedRawEpoch, from the driver's GnssTimeSync). It is held until the IMU
// has passed that instant, then built into measurements (RawGnssMeasStage:
// RawMeasBuilder, with the ephemerides the receiver has sent so far) and
// applied with its age: the receiver is taken back to the epoch by the IMU's
// own velocity history, however late the epoch arrived. One later than
// max_epoch_age_s, past that history, is dropped. An epoch with no MCU time
// yet (the time sync wants 10 s of fixes) only counts for the satellites'
// relock ages.
//
// Built elsewhere (Config::external_build, #1626). On the flight computer the
// build -- two-thirds of an epoch's cost -- runs in a GNSS task of its own,
// below this one's priority, with a stage of its own (RawGnssMeasStage.h):
// the epochs come here built (onBuilt), the satellites the checks find jumped
// go back to it (takeRelock), and its counts come with them (onMeasStats).
// The ephemerides still come here, for the pad context only. Without it, as a
// host test drives it, the shadow builds in its own job.
//
// The ionosphere (phase 5). A pad calibration brings the GPS broadcast
// (Klobuchar) parameters from the app, with the date of the navigation file
// they came from (setIono; NVS keeps them across boots). They are used while
// they are no older than iono_max_age_days by the receiver's date. Otherwise,
// and without them, the pad survey's per-satellite corrections are: on the
// pad, against the launch site (PadSurvey.h), and frozen at liftoff -- when the
// raw measurements vouch for the launch site (the receiver's fix can be
// hundreds of metres out). The choice is made on the pad and kept through the
// flight; the survey runs either way, and starts over when the choice changes.
//
// Physics checks (#1508). Each raw epoch, every satellite's pseudorange is
// set against the IMU's propagation since the last one (SatJumpCheck.h): one
// that jumped is taken as relocked, and the filter de-weights it until it
// settles. Each of the receiver's own fixes, held until the IMU passes its
// GPS instant, is set against what the IMU predicts from the last fix that
// passed (FixCheck.h) and logged (GNSS_CHECK_MSG). That one only reports: the
// flight filter takes every fix as before.
//
// Barometer and landing. Each barometer sample the flight filter fuses is
// fused here too, as a height: the barometer's height above the pad plus an
// offset to the filter's own (ellipsoidal) height, learned on the pad while
// the raw measurements lead and frozen at launch. Its variance is the flight
// filter's: (2 m)^2 plus the static port's error at the dynamic pressure
// (#1579). With no satellites -- the receiver lost the sky under the main on
// the rig, and with nothing else the shadow drifted 70 m in 20 s -- it holds
// the vertical. Once the landing detector has called it, zero velocity, 10
// times a second (#1418).
//
// The navigation filter (Config::primary: the Beetle, TR_RAW_GNSS_PRIMARY).
// There GpsInsEKF does not run, and the seed is no filter's state, only where
// to start: the receiver's pad position, an attitude from the accelerometer,
// the still gyro's mean, the declination and the field's size at the pad. So
// on the ground the filter keeps its own attitude -- it levels, and takes the
// magnetometer as a heading (#1578's measurement) -- and is never resynced.
// The magnetometer only with its hard iron out (#1590): each sample says
// whether a stored calibration is applied, or the field is a sim's
// (ShadowMag::flags); any other is refused and counted (mag_uncal), as is one
// off the model's size (mag_suspect, #1304) or, against the accelerometer's
// down, off the model's dip (mag_dip, mag_dip_tol_deg).
// It learns the barometer's offset with or without a fix of its own (a flight
// without GNSS has the barometer and the IMU). A seed marked restore -- a
// flight the flight computer restored after a reboot -- starts it in flight,
// the barometer's offset taken from its first sample; with restore_*_sigma,
// its position, velocity and attitude declared that unsure (the reboot's gap
// was flown on nothing), and with restore_seed_bias the snapshot's
// accelerometer bias in place of zero. Its state goes to the flight loop
// every step (navState, ShadowLink::nav).
//
// Levelling. Where the flight filter takes its accelerometer as gravity (the
// ground, and after apogee) the shadow does too -- in flight only once its
// own velocity has held steady from epoch to epoch and the specific force is
// g's size (LevelGate.h): as a drogue opens, the drag is far off vertical.
// Gravity carries no heading, and a levelling never turns it (the EKF's
// level_heading_free): act's wide launch heading, correlated with the tilt
// by the boost, is otherwise turned by the first levelling under the chute.
//
// The heading from thrust (HeadingFit.h, heading_mode). Near vertical
// nothing the filter fuses sees its heading. The fit is seeded at the last
// still step on the pad (the filter's attitude and gyro bias, and the IMU
// biases as the pad left them), runs on the gyro alone from there, and after
// launch takes each raw epoch's Doppler velocity -- an epoch's work gains a
// step for it, between the checks and the receiver clock -- logging each fit
// (HEADING_FIT_MSG). Observe: that is all. Act: from launch the filter's
// heading has the launch prior (below: the magnetometer's, not the pad's
// near-certainty), its tilt at least heading_launch_tilt_sigma_deg (0.6
// deg; the pad cannot tell its tilt from an accelerometer bias, #1626 K1),
// and GNSS reaches all of its attitude, neither att_gate_vertical nor cos^4
// pitch, from launch -- or only about the vertical, from launch or until the
// flight computer's apogee call (heading_gate, HeadingGate; the call rides
// each IMU tick, kShadowApogee, and a restored flight comes back in the gate
// its phase has); back on the ground both are the EKF config's again. All of
// it from launch is the default since 2026-10-16: on the RF rig's three skies
// (rp5, Eagle Claw, GTV; magnetometer 0, +10, +40) it matched the heading
// alone until the apogee call before apogee, ended a 40 deg error at 0.4-2.1
// deg against 3.3-6.0, and moved neither apogee vote. Once the fit's 1-sigma
// is heading_handoff_deg or less (never by default: below),
// and no other heading far off fits within heading_handoff_rival of it (the
// profile's second valley, HeadingFit::Result::rival -- for thrust along the
// nose there is none), its heading is handed over, once a flight -- turned
// onto the fit's, the IMU biases put back as the pad left them, the velocity
// let go to heading_handoff_vel_mps and the satellites refused under the old
// heading forgotten -- before that epoch is fused. The hand-off also waits
// for the filter's heading to be further from the fit's than
// heading_handoff_apart (2) of their combined 1-sigmas -- the filter lost,
// not just unsure -- until the fit is sure to heading_handoff_sure_deg (3
// deg); and, if asked, for heading_handoff_min_epochs. (On the rig's r18 act's
// own GNSS had already righted a 40 deg error to 2.5 when a two-epoch fit at
// a 1-sigma of 5 deg, 9.8 deg off, was handed over.) The epoch's two units
// (its Doppler velocity, then the fit) are each one slice at most. The
// hand-off is off by default (heading_handoff_deg below 0): it overwrites
// the filter's heading rather than fusing a measurement (r18 above), and the
// filter's own GNSS updates turn the heading anyway.
//
// Act's launch prior: the heading's 1-sigma from the launch edge on. The
// magnetometer's heading, calibrated, is good to about 4-5 deg on the Beetle
// in its flight configuration (10 until that is measured again); a wider
// prior only lets a thin boost epoch throw a right heading about -- in the
// offline replay a 3-satellite epoch moved one 10-15 deg at a 30 deg prior
// and a magnetometer error of 0, at 10 deg 2.6 at most. So where the pad's
// heading is the magnetometer's -- a sample through the calibration, size
// and dip gates (applyMag) fused within heading_mag_recent_s of the launch
// edge -- the prior is heading_pad_sigma_deg (10); otherwise
// heading_pad_sigma_fallback_deg (30). The window: the magnetometer is
// taken only on the ground at g's size, so the last sample before the edge
// is at least the ignition-to-verdict delay old (0.1-0.4 s on the rig), and a
// start that keeps the force off g's size or an igniter's current that the
// gates refuse can add a second or two; a magnetometer silent or refused for
// longer than 5 s before launch is no longer the heading's source (the
// gyro's drift over that time is a few tenths of a degree at most, but
// whatever stopped the gates passing it is unexplained). The shadow (not
// the navigation filter) takes no magnetometer of its own: the fallback.
//
// A launch with no magnetometer heading (#1590). Where the pad fused none --
// no magnetometer, or none calibrated -- the heading at launch is the start's
// guess (main.cpp's PAD_HEADING_DEG about the vertical), its 1-sigma p0_hdg's
// 57 deg grown to ~76 over a long pad by the vertical gyro bias's. Act's
// fallback prior is then claimed about a heading nothing measured. Each
// launch's own heading 1-sigma (before act's) is kept in the stats, and a
// launch with no magnetometer heading counted (heading_unset_launches), for
// the flight computer to say loudly.
//
// The boost screen (boost_screen, boost_screen_g). A receiver losing lock in
// the boost can hand over an epoch whose Dopplers are tens of m/s off: on the
// rig (c30) the LC86G, at 3.8 g at T+1 s, gave one with Dopplers 36-70 m/s
// out. Act's opened heading and the boost's heading-velocity correlation
// (+-0.83) widened the innovation gate: it refused 4 of the 7, let 3 through,
// and those turned the heading +22.8 deg for the rest of the flight (GNSS was
// gone after it). So, before the apogee call, an epoch whose start finds the
// IMU past boost_screen_g (2; 0 judges every epoch) within the last
// boost_screen_window_s (1.5 s) is a boost epoch -- the window, not the peak
// since the last epoch, so the first epoch back after a dropout is no boost
// epoch: a filter 6 s without GNSS refuses its rows, and the screen would drop
// the epoch that rights it -- and at the start of its fusion, after the
// receiver clock's step and before any update, it is judged, none of it
// fused:
//   - its Dopplers gated against the state as it stands, each with the test
//     the update would give it as the epoch's first (TR_RawGnssEKF::
//     screenRangeRates);
//   - and, unless that already drops it, the epoch worked through on a
//     scratch copy of the filter (dry_), a satellite a slice, with the
//     update's own code -- its pseudoranges and Dopplers in the update's
//     order -- and its Dopplers' refusals counted. A bad Doppler the gate
//     passes at the prior pulls the state and the good ones after it are
//     refused (a cascade), and every update shrinks the gate for the rest:
//     the prior's gate alone kept c30-like epochs in the offline replay
//     (3 of 7 refused there, 4-5 of 7 worked through) that turned the heading
//     9 deg past unaided's. The two together drop what the replay's majority
//     check dropped, epoch for epoch.
// If either refuses half or more of the Dopplers the whole epoch is dropped:
// no pseudorange, Doppler or carrier update, and it is no satellite check's
// reference, no levelling epoch and no carrier rate's earlier end. Half or
// more, not most: in the offline replay a strict majority let ties through.
// Only the Dopplers count, never the carrier's rates (none in a boost anyway).
// After the apogee call it never runs: a drogue's snatch can pass 2 g, and the
// epoch after it can be the carrier's first. By default it runs in act
// (kScreenAct), where GNSS turns the attitude from launch. Not on every
// epoch: it judges the data by the filter, so a filter itself wrong -- a
// magnetometer 40 deg off out of the boost, sure of its heading -- sees
// good Dopplers refused and the epochs that would right it dropped (the
// cloud session's 16-flight grid, 2026-10-16: one run in 32 at +-40 still 25
// deg off at T+14 against 7; 11 in 32 dropped epochs on r18's boost).
//
// The vertical check (vert_check_mps; off at 0). What the rig's receiver
// hands over after losing its loops is coherent, not scattered: c30's T+1
// Dopplers are those of a receiver still on the pad, within 0.8 m/s of one
// velocity that is 82 m/s down of the filter's; GTV's T+1 is 67 m/s off
// and recovers over six epochs (27, 17, 11, 9, 6, 3); a re-flown Eagle
// Claw's T+2 is 54. So each epoch with four Dopplers or more has one
// velocity and clock rate fitted to its Dopplers (TR_RawGnssEKF::
// screenRangeRates), and is dropped whole -- as the boost screen drops one
// -- if the fit's vertical is more than vert_check_mps off the filter's.
// The vertical is the IMU's and the barometer's whatever the heading, so a
// filter wrong about its heading is never locked out, and it runs on every
// epoch, in every phase, the boost's length whatever it is. Good epochs on
// the rig came within 1.3 m/s. On by default (20 m/s) since 2026-10-16,
// beside the boost screen, which still judges every boost epoch: an epoch is
// dropped if either fails. Each sees what the other misses -- a stale boost
// epoch whose Dopplers agree with one another passes the boost screen's gate
// (in the cloud session's grid, a c30-type epoch then no GNSS to T+8: 3 runs
// in 112 over 3 deg worse than unaided with the boost screen alone, none
// with the vertical check), and one with exactly four Dopplers has no spare
// row to show its vertical (7 m/s down where the boost screen refused 3 of
// 4).
//
// The vertical check's latch (vert_release_mps; 5 m/s, none at 0). A
// receiver that lost its loops can come back over seconds, not at once, and
// the epochs on its way back fit under vert_check_mps: in the cloud
// session's grid (2026-10-16, the firmware's clock; a whole receiver stale,
// closing with tau = 2 s from burnout) one fitting +19.8 m/s down was fused
// and turned the heading 18 deg, for the rest of the flight where GNSS was
// lost after it. So after a vertical drop every epoch is dropped too until
// one's fit is back within vert_release_mps, and one too thin to fit (under
// four Dopplers) is dropped while it holds. On that grid, with GNSS after
// the tail, it cut the runs over 3 deg worse than unaided from 43 to 31 of
// 112 and the worst from 29.7 to 14.0 deg; with no fault it changed
// nothing. The rig's receivers came back within 1 m/s at the next epoch or
// with a 3-5 m/s tail (eca10, f4b), released at once; of 24 rig logs it
// changed only v10, whose 2-3 Doppler epochs after its T+1 drop had turned
// a right heading 9 deg.
//
// The carrier range rate (the receiver profile's cr_mode, ekf.cr_mode). Each
// epoch's builder gives the satellites whose carrier phase was continuous
// since the epoch before a carrier range rate (RawMeasBuilder), and the
// filter screens them after the epoch's pseudoranges and Dopplers
// (TR_RawGnssEKF.h). Observe (the LC86G's): the screen, the innovations and
// the epoch's common rate are computed and counted at the epoch's close --
// in the unit that closes it, so no step or slice falls differently -- and
// the state is never touched. Fuse: a unit screens, the next fuses every
// screened rate at once (the common rate is the state's at that instant).
// The counts, the innovations' RMS, the common rate (m/s, and in cycles a
// second: with the roll rate beside it, the carrier wind-up) are in the
// stats and on the console.
//
// A receiver fused faster than 1 Hz (the mosaic-G5's 10 Hz). The safeguards
// above were set on the LC86G's epoch a second, and some count epochs, so a
// faster profile keeps their time instead:
//   - the pseudoranges are fused once in each pr_period_s of GPS time (1 s),
//     on the first epoch of it that is fused: each code update's correlated
//     part is inflated by the time since that satellite's last, so fused
//     every 0.1 s the gate would open sqrt(10) wider for no more information.
//     An epoch lost, stale or dropped on the whole second moves the code to
//     the next one rather than skipping the second. gnss_aided still means a
//     pseudorange fused within 2 s;
//   - a Doppler refused the EKF's reject_persist_rr epochs running is let in
//     (30 at 10 Hz: the Beetle's 3 s);
//   - the Doppler's stress takes the IMU's peak over rr_dyn_window_s (1 s,
//     the LC86G's epoch), not the 0.1 s since the epoch before: the largest
//     any epoch within the window carried, so a loop still settling after
//     burnout is weighted as the Beetle weighs it for the whole second,
//     however the thrust fell before it;
//   - the levelling's steady test takes epochs LevelGate's min_dt_s apart, and
//     the heading fit one epoch a heading_fit_period_s (its 64 would fill in
//     6.4 s);
//   - with level_on_fused_epochs an epoch that fused nothing is no levelling
//     epoch: under a chute with the sky gone the mosaic still sends its empty
//     epochs, and the outage's levelling would never start.
//
// The turning body. The filter takes the antenna at the IMU (r = 0), and the
// offset between them is not measured before a flight. While the body turns
// at omega, an antenna r away moves omega x r against the IMU: 1.5 m/s at the
// drogue's snatch (5 rad/s) with 0.3 m on the nose, a velocity every Doppler
// sees and the IMU's does not, which the update reads as an attitude error
// (Grid B, 2026-10-17: up to 42 deg in the snatch's first 3 s, no fault). Two
// allowances, each off at 0, need only a bound on r and the gyro:
//   - each Doppler's sigma takes, in quadrature, the most omega x r can be
//     (rotLeverSigma): rot_lever_axial_m (along the nose) times the rate
//     across the nose, plus rot_lever_radial_m (across it) times the whole
//     rate -- an antenna on the axis does not move as the body rolls, one off
//     it does. The rate is the gyro less its bias at its largest since the
//     epoch before, so the epoch's own instant is in it. The boost screen,
//     the vertical check and the update all judge the Dopplers with it;
//   - an epoch whose rate across the nose reached att_hold_rate_rps, and each
//     within att_hold_s after one, keeps GNSS off the attitude and the gyro
//     bias (the unaided twin's mask, for those epochs): the snatch and a
//     tumble. Roll alone never holds it: a spinning rocket keeps its GNSS
//     attitude.
// With rot_after_apogee both act only once the flight computer has called
// apogee: the drogue's snatch and the descent. The boost and the coast --
// where a boost that weathercocks would be held, and where the votes and the
// raw vote guard read the filter -- fly as without them.
//
// Stopping. A sensor sim, a hole in the IMU longer than max_gap_s, a state
// that is not finite, or (on the pad) one pad_reset_m from the flight
// filter's stops it; it starts again from the next pad seed.
//
// Rate. It steps at most every min_step_s (25 Hz), on the time-weighted mean
// of the flight filter's ticks since its last step: the flight filter ticks
// 150-400 times a second on the Beetle, and every step this filter takes on
// core 0 is code fetched through the cache and flash bus the flight loop on
// core 1 shares (2026-10-13, at IDF's DIO and 16 KB cache: stepping on every
// tick cost the flight loop 10 % of its rate; see main.cpp for the cost now).
//
// CPU. With a clock (setClock: esp_timer on the flight computer) it times
// each IMU step and each epoch's build and update, into its stats and its
// log records. Pure C++, no FreeRTOS: the flight computer's core-0 task feeds
// it (main.cpp, raw-GNSS shadow), and tests_cpp/test_raw_gnss_shadow.cpp
// drives it directly.
#ifndef RAW_GNSS_SHADOW_H
#define RAW_GNSS_SHADOW_H

#include <cstddef>
#include <cstdint>

#include <RocketComputerTypes.h>

#include "EphemerisFrames.h"
#include "EphemerisStore.h"
#include "FixCheck.h"
#include "HeadingFit.h"
#include "LevelGate.h"
#include "PadSurvey.h"
#include "RawGnssMeasStage.h"
#include "RawMeas.h"
#include "RawMeasBuilder.h"
#include "SatJumpCheck.h"
#include "ShadowLink.h"
#include "TR_RawGnssEKF.h"

namespace gnss_raw
{

class RawGnssShadow
{
public:
    struct Config
    {
        RawGnssEKFParams       ekf;
        RawMeasBuilder::Config meas;
        double   epoch_latency_s = 0.0;   // the receiver's delay the time sync leaves in (its
                                          // latency_s, uncalibrated: 0)
        double   max_epoch_age_s = 0.6;   // later than this an epoch is dropped
        double   epoch_slice_us  = 0.0;   // the navigation filter: an epoch's work at most this long a step (0: at once)
        double   epoch_job_max_s = 0.3;   //   but all of it once the epoch is this old (the IMU history is 0.5 s)
        double   log_period_s    = 0.1;   // RAW_GNSS_EKF_MSG at 10 Hz
        double   min_step_s      = 0.04;  // the filter steps at most this often (25 Hz), on
                                          // the mean of the ticks since; 0 = every tick
        double   resync_period_s = 1.0;   // the pad attitude retaken this often
        double   seed_max_age_s  = 2.0;   // a seed older than this is not started from
        double   max_gap_s       = 0.5;   // an IMU hole longer than this stops it
        double   pad_reset_m     = 500.0; // on the pad, this far from the flight filter stops it
        uint16_t default_week    = 2440;  // until an epoch names the GPS week (2026-10)
        int      iono_max_age_days = 30;  // broadcast parameters older than this give way to the pad survey
        PadSurvey::Config survey;
        SatJumpCheck::Config sat_check;
        FixCheck::Config fix_check;
        double   acc_window_s = 0.08;     // the IMU's acceleration at a fix: its mean over this long
        LevelGate::Config level;          // when it levels in flight
        double   baro_sigma_m    = 2.0;       // the barometer's own noise, m (the flight filter's)
        double   baro_port_k     = 0.0075;    // the static port's error, m per Pa of dynamic pressure (#1579)
        double   baro_offset_tau_s = 30.0;    // the pad offset follows the filter's height this slowly
        double   baro_offset_min_s = 10.0;    // and is used once it has had this long
        double   baro_hold_port_m = 2.0;      // in flight: held out while that error at the vertical speed is above this
        double   baro_hold_max_s  = 30.0;     //   but no longer than this after launch
        double   zupt_sigma_mps  = 0.5;       // landed: zero velocity, 1 sigma (#1418)
        double   zupt_period_s   = 0.1;       //   this often
        bool     pad_zupt        = true;      // the navigation filter, on the pad while still: zero velocity too
        double   still_rate_rps  = 0.05;      //   still: rotating slower than this, the force g's size
        bool     primary         = false;     // the navigation filter (above)
        double   mag_field_tol   = 0.15;      // a field this far off the model's size is no heading (#1304)
        // A field whose dip -- its angle below the horizontal, the
        // accelerometer's down the vertical -- is further than this from the
        // model's is no heading either (#1590; 0: not checked). What a
        // calibrated QMC5883P's direction can be off by, worst case: 1 uT of
        // hard iron the calibration left, at the ~45 uT the Beetle flies in,
        // 1.3 deg; about 1 deg of axis non-orthogonality; the soft iron a
        // hard-iron calibration leaves, a few % between axes (3 %: 0.9 deg);
        // the accelerometer as the vertical (mag_tilt_sigma_mps2's 0.05 m/s^2:
        // 0.3 deg); the model's dip and a local anomaly, 0.5 deg. Summed 4.0
        // deg (2.0 root-sum-square), and the sample's own noise (0.5 uT an
        // axis) 0.4 deg rms, 1.2 at three sigma: 5.2. 6 passes all of it.
        // Simulated at random attitudes in the fields of New Jersey and New
        // Mexico (1 uT, 2 % gains, 1 deg, 0.3 deg, 0.5 uT), no sample was
        // past 5.5 deg; with the gains the Beetle's bench fit found left
        // uncorrected (1.07/0.99/0.94), up to 0.03 % were past 6. The hard
        // iron the size test lets through mostly tips the dip far more: of
        // the attitudes where an uncalibrated Beetle's 40 uT passes it and
        // reads a heading over 10 deg off (16-18 %), this leaves a sixth to
        // a quarter (3-4 %). It cannot see a hard iron that only turns the
        // field about the vertical -- size and dip both kept -- so it does
        // not stand in for the calibration.
        double   mag_dip_tol_deg = 6.0;
        double   max_bridge_s    = 5.0;       // the navigation filter bridges an IMU hole this long
        double   pad_context_delay_s = 6.0;   // the pad context is written from this long after launch
        bool     external_build = false;      // the epochs come built (onBuilt): a GNSS task builds them
        // the heading from thrust (above)
        uint8_t  heading_mode = 0;            // HeadingMode
        HeadingFit::Config heading_fit;
        double   heading_seed_max_s = 3.0;    // a seed this long before launch is too old (not still at ignition)
        double   heading_fit_max_s  = 60.0;   // epochs taken this long after launch
        double   heading_handoff_deg = -1.0;  // act: the fit's 1-sigma at or under this hands its heading over
                                              //   (below 0, the default: never)
        double   heading_handoff_rival = 25.0;   // act: ... unless another heading's valley rises less than this
                                                 //   above the fit's (as its sigma's: 25 is five sigmas')
        int      heading_handoff_min_epochs = 0; // act: ... once the fit holds this many epochs
        double   heading_handoff_apart = 2.0;    // act: ... and only while its heading and the filter's are more
                                                 //   than this many of their combined 1-sigmas apart: the filter
                                                 //   lost, not just unsure (0: whatever the filter's) --
        double   heading_handoff_sure_deg = 3.0; //   unless the fit's 1-sigma is at or under this
        double   heading_pad_sigma_deg = 10.0;   // act: the heading's 1-sigma from launch on, where the pad's is
                                                 //   the magnetometer's (above)
        double   heading_pad_sigma_fallback_deg = 30.0;   // act: and where it is not
        double   heading_mag_recent_s = 5.0;     // act: the magnetometer's when it fused within this of the launch edge
        double   heading_launch_tilt_sigma_deg = 0.6;   // act: and the tilt's at least this from launch
                                                        //   (the pad's is the accelerometer bias's too,
                                                        //   #1626 K1; 0: off)
        uint8_t  heading_gate = 1;               // act: what of the attitude GNSS reaches in flight, and
                                                 //   when each opens (HeadingGate: all of it from launch)
        double   heading_handoff_vel_mps = 3.0;  // act: the velocity let go to this at the hand-off
        // the boost screen (above)
        uint8_t  boost_screen = 1;               // BoostScreen: off, in act only (the default), always
        double   boost_screen_g = 2.0;           // a boost epoch: the IMU's specific force above this many g,
                                                 //   before the apogee call (0: every epoch) ...
        double   boost_screen_window_s = 1.5;    //   ... within this long before the epoch's start
        // the vertical check (above)
        double   vert_check_mps = 20.0;          // an epoch whose Dopplers' fitted vertical is more than this off
                                                 //   the filter's is dropped (0: off)
        double   vert_release_mps = 5.0;         //   and, after one, every epoch after it until one's fitted
                                                 //   vertical is back within this (0: no latch)
        // A receiver fused faster than 1 Hz (below; the mosaic-G5's profile
        // sets them, ReceiverProfileApply.h). 0 or false: an epoch at a time,
        // as the Beetle's LC86G flies.
        double   pr_period_s = 0.0;              // the pseudoranges fused once in each this long of GPS time
        double   rr_dyn_window_s = 0.0;          // the Doppler's stress (meas.rr_dyn_sigma) on the IMU's peak over
                                                 //   this long, not only since the epoch before
        double   heading_fit_period_s = 0.0;     // the heading fit takes an epoch at most this often
        bool     level_on_fused_epochs = false;  // the levelling's steady test takes only epochs that fused something
        int      sat_cap = 0;                    // the GNSS task's stage: at most this many satellites built an
                                                 //   epoch (0: all)
        // A flight restored after a reboot (the navigation filter): its
        // start's 1-sigmas, every axis (0: the start's own -- the attitude the
        // seed's, known), and the snapshot's accelerometer bias
        // (ShadowSeed::has_accel_bias)
        double   restore_pos_sigma_m = 0.0;
        double   restore_vel_sigma_mps = 0.0;
        double   restore_att_sigma_deg = 0.0;
        bool     restore_seed_bias = false;
        // An antenna away from the IMU by an offset nobody measured (above:
        // the turning body). 0: off, the Beetle's filter exactly.
        double   rot_lever_axial_m = 0.0;     // its offset along the nose at most, m
        double   rot_lever_radial_m = 0.0;    //   and across it, m: each Doppler's sigma takes rotLeverSigma
        double   att_hold_rate_rps = 0.0;     // an epoch whose rate across the nose reached this, rad/s, and
        double   att_hold_s = 0.0;            //   every epoch within this long after one: GNSS held off the
                                              //   attitude and the gyro bias (RawGnssEKF::setAttHold)
        bool     rot_after_apogee = false;    // both only after the flight computer's apogee call
    };

    // When the boost screen runs (Config::boost_screen).
    enum BoostScreen : uint8_t
    {
        kScreenOff = 0,
        kScreenAct = 1,        // in act only: GNSS turns the heading from launch (the default)
        kScreenOn = 2,         // whatever the heading mode
    };

    enum HeadingMode : uint8_t
    {
        kHeadingOff = 0,
        kHeadingObserve = 1,   // fit and log
        kHeadingAct = 2,       // and hand the heading over (the filter gated about the vertical)
    };

    // Act's attitude gate in flight (Config::heading_gate): what of the
    // attitude a GNSS correction reaches, and when each opens. Back on the
    // ground, the EKF config's (cos^4 pitch) again; observe and off never
    // touch it.
    enum HeadingGate : uint8_t
    {
        kGateHeading = 0,      // the heading only (att_gate_vertical), from launch (C)
        kGateFullLaunch = 1,   // the whole attitude -- neither that gate nor cos^4 -- from launch (A, the default)
        kGateFullApogee = 2,   // the heading only from launch, the whole attitude from the flight
                               //   computer's apogee call (kShadowApogee) to the ground (H)
    };
    static const char* headingGateName(uint8_t gate)
    {
        switch (gate)
        {
        case kGateHeading:    return "the heading's from launch";
        case kGateFullLaunch: return "FULL from launch -- GNSS turns the tilt too";
        case kGateFullApogee: return "the heading's from launch, FULL from the apogee call";
        default:              return "?";
        }
    }

    // What takes the ionosphere out (the pad status message reports it).
    enum IonoSource : uint8_t
    {
        kIonoNone = 0,       // neither: Klobuchar's night-time 5 ns only
        kIonoBroadcast = 1,  // the GPS broadcast parameters a pad calibration brought
        kIonoPad = 2,        // the pad survey's corrections (some in use, the pad vouched for)
    };

    enum StopReason : uint8_t
    {
        kStopNone = 0, kStopSim, kStopGap, kStopNotFinite, kStopFar
    };

    struct Stats
    {
        uint32_t imu = 0;               // IMU ticks seen
        uint32_t imu_repeats = 0;       //   with the previous one's time (skipped)
        uint32_t starts = 0, stops = 0;
        uint8_t  last_stop = kStopNone;
        uint32_t epochs = 0;            // raw epochs seen
        uint32_t epochs_untimed = 0;    //   with no MCU time yet
        uint32_t epochs_idle = 0;       //   while not running
        uint32_t epochs_stale = 0;      //   dropped as too late
        uint32_t epochs_applied = 0;
        uint32_t eph_added = 0, eph_bad = 0;
        uint32_t resyncs = 0;
        // CPU, with a clock (us)
        uint64_t step_us_sum = 0;
        uint32_t step_n = 0, step_us_max = 0;
        uint32_t build_us = 0, update_us = 0;            // the last epoch
        uint32_t build_us_max = 0, update_us_max = 0;
        uint32_t epoch_slices = 0;         // the last epoch's work, in this many slices
        uint32_t epoch_wall_us = 0;        //   from its first to its last (the filter stepping in between)
        uint32_t slice_us_max = 0;         // the longest slice
        // the last epoch applied
        RawGnssStats last;
        double   last_age_s = 0.0;
        uint32_t last_tow_ms = 0;
        int      last_meas = 0;
        int      last_pad_corrected = 0;   // of its pseudoranges, corrected by the pad survey
        // the last epoch seen: GPS, Galileo and BeiDou satellites with a
        // pseudorange, and of those, the ones with no ephemeris yet
        int      tracked = 0;
        int      tracked_no_eph = 0;
        uint32_t survey_epochs = 0;        // epochs the pad survey took
        uint32_t iono_sets = 0;            // setIono calls
        // the physics checks
        int      last_sats_jumped = 0;     // the last epoch's satellites taken as jumped
        uint32_t sats_jumped = 0;          //   all of them
        uint32_t fixes = 0;                // receiver fixes seen
        uint32_t fixes_untimed = 0;        //   with no GPS instant on the MCU clock yet
        uint32_t fixes_dropped = 0;        //   more waiting than the queue holds
        // levelling on the accelerometer, where the flight filter levels
        uint32_t levels = 0;               // steps that levelled
        uint32_t levels_held = 0;          //   in flight, held off: not steady, or not g's size
        // the barometer and the landing
        uint32_t baro = 0;                 // samples seen
        uint32_t baro_fused = 0;           //   fused (an offset learned)
        uint32_t baro_held = 0;            //   held out in flight at speed (the port's error)
        double   baro_offset_m = 0.0;      // the filter's height less the barometer's, on the pad
        uint8_t  baro_offset_frozen = 0;   //   frozen at launch
        uint32_t zupts = 0;                // landed zero-velocity updates
        uint32_t pad_zupts = 0;            //   and on the pad, still
        // the magnetometer (the navigation filter, on the ground)
        uint32_t mags = 0;                 // samples seen
        uint32_t mag_fused = 0;            //   taken as a heading
        uint32_t mag_uncal = 0;            //   refused: no stored hard-iron calibration applied (#1590)
        uint32_t mag_suspect = 0;          //   refused: not the model's size
        uint32_t mag_dip = 0;              //   refused: not the model's dip (mag_dip_tol_deg)
        float    mag_dip_err_deg = 0.0f;   // the last sample the dip was taken of: its dip less the model's
        uint32_t restores = 0;             // starts in flight from a restored flight
        uint32_t gaps_bridged = 0;         // IMU holes the navigation filter stepped over
        uint32_t pad_contexts = 0;         // launches the pad context was taken at
        // the heading from thrust
        uint32_t heading_seeds = 0;        // still pad steps it was seeded at
        uint32_t heading_flights = 0;      // launches it ran from (its seed fresh enough)
        uint32_t heading_epochs = 0;       // epochs it took
        uint32_t heading_vel_fails = 0;    //   with no Doppler velocity (too few satellites agreeing)
        uint32_t heading_unset_launches = 0;  // launches whose pad fused no magnetometer heading (#1590)
        float    heading_launch_sigma_deg = 0.0f;   // the last launch's own heading 1-sigma, before act's prior
        float    heading_launch_mag_age_s = -1.0f;  //   its last magnetometer heading, this long before (-1: none)
        uint32_t heading_act_launches = 0; // act: launch edges that set the prior
        uint32_t heading_mag_launches = 0; //   of them, with the magnetometer's heading (heading_pad_sigma_deg)
        float    heading_launch_prior_deg = 0.0f;   //   the last one's prior,
        uint8_t  heading_launch_mag = 0;   //   1: the magnetometer's (heading_pad_sigma_deg), 0: the fallback
        uint32_t heading_gate_apogee = 0;  // act: full gates opened at the apogee call (kGateFullApogee)
        float    heading_gate_apogee_s = 0.0f;   //   the last, after the launch edge (-1: a restored flight's)
        uint32_t heading_handoffs = 0;     // headings handed over
        uint32_t heading_rivals = 0;       // fits another heading's valley came within heading_handoff_rival of
        float    heading_turn_deg = 0.0f;  //   the last one's turn
        float    heading_handoff_s = 0.0f; //   and when, after launch
        HeadingFit::Result heading_last;   // the last fit
        uint32_t heading_vel_us = 0, heading_vel_us_max = 0;      // an epoch's Doppler velocity
        uint32_t heading_solve_us = 0, heading_solve_us_max = 0;  //   and its fit
        uint32_t heading_step_us_max = 0;  // the fit's share of a step
        // the carrier range rates (above): totals, each epoch's in last
        uint32_t cr_epochs = 0;            // epochs with any
        uint32_t cr_too_few = 0;           //   with under 4 to screen: none taken
        uint32_t cr_built = 0;             // rates offered
        uint32_t cr_unmatched = 0;         //   whose earlier epoch the filter did not keep
        uint32_t cr_in = 0;                //   screened in (repaired ones too)
        uint32_t cr_half = 0;              //   rejected as half-cycle steps
        uint32_t cr_repaired = 0;          //   repaired
        uint32_t cr_other = 0;             //   rejected otherwise
        uint32_t cr_used = 0;              //   fused (fuse mode)
        float    cr_last_rms_mps = 0.0f;   // the last epoch screened: its innovations' RMS about the common rate,
        float    cr_last_fit_rms_mps = 0.0f;   //   and about the screen's fit,
        float    cr_last_dv_mps = 0.0f;    //   the velocity correction the rates point to,
        double   cr_last_common_mps = 0.0; //   the common rate,
        float    cr_last_roll_rev_s = 0.0f;//   the mean |roll rate|
        // the boost screen
        uint32_t boost_screened = 0;       // boost epochs screened
        uint32_t boost_dropped = 0;        //   dropped: half or more of their Dopplers refused
        uint32_t boost_dropped_dry = 0;    //     of them, by the scratch copy's updates (the prior's gate kept them)
        int      boost_last_rr = 0;        // the last one screened: its Dopplers
        int      boost_last_refused = 0;   //   refused at the prior,
        int      boost_last_dry_used = 0;  //   fused on the scratch copy (-1: not worked through)
        int      boost_last_dry_refused = 0;   //   and refused there
        float    boost_last_g = 0.0f;      //   its IMU peak since the epoch before, g
        // the vertical check
        uint32_t vert_checked = 0;         // epochs with a velocity fitted to their Dopplers
        uint32_t vert_dropped = 0;         //   dropped: the fit's vertical more than vert_check_mps off
        float    vert_last_dv_down = 0.0f; //   the last one's: its vertical against the filter's, m/s
        float    vert_worst_dv_down = 0.0f;//   and the largest kept (|.|), m/s
        uint32_t vert_latched = 0;         // epochs dropped after a vertical drop, not yet back (vert_release_mps)
        // the turning body (rot_lever_*, att_hold_*)
        float    rot_last_sigma_mps = 0.0f;   // the last epoch's rotation allowance on its Dopplers
        float    rot_max_sigma_mps = 0.0f;    //   the largest any epoch took
        float    rot_last_across_rps = 0.0f;  // the last epoch's largest rate across the nose
        uint32_t att_held = 0;             // epochs whose GNSS was held off the attitude (att_hold_rate_rps)
        bool     att_last_held = false;    //   the last one's
    };

    using Clock = int64_t (*)();

    RawGnssShadow();
    explicit RawGnssShadow(const Config& cfg);
    // What a GNSS task's stage builds with, for a shadow with this config.
    static RawGnssMeasStage::Config stageConfig(const Config& c);
    // The most an antenna up to axial_m along the nose and radial_m across it
    // can move against the IMU while the body turns at rate_rps, of which
    // across_rps across the nose (m/s): |omega x r| <= axial |omega across|
    // + radial |omega| (along the nose only the cross rate moves it; across,
    // any rate does, up to its whole size). Below 0 counts as 0.
    static double rotLeverSigma(double axial_m, double radial_m, double rate_rps, double across_rps);
    void setClock(Clock c) { clock_ = c; }
    const Config& config() const { return cfg_; }

    // Stop; ephemerides and relock history are kept.
    void reset();

    void onSeed(const ShadowSeed& s);
    // A broadcast ephemeris (RTCM3 payload): kept for the pad context, and
    // decoded for the build unless a GNSS task builds (external_build).
    void onEphemeris(const uint8_t* payload, size_t len);
    // The broadcast ionosphere a pad calibration brought (or NVS kept).
    void setIono(const ShadowIono& io);
    // One of the receiver's own fixes (FixCheck).
    void onFix(const TimedFix& f);
    // A barometer sample the flight filter fused; applied at the next step.
    void onBaro(const ShadowBaro& b);
    // A magnetometer sample; the navigation filter takes it on the ground.
    void onMag(const ShadowMag& m);
    // The navigation state the flight loop flies on (running = 0 before a start).
    void navState(ShadowNav& out) const;
    // A raw epoch, built here (not with external_build).
    void onEpoch(const TimedRawEpoch& e);
    // external_build: an epoch the GNSS task built, held as an epoch is.
    void onBuilt(const BuiltEpoch& b);
    // external_build: a satellite the checks found jumped, for the GNSS
    // task's stage (RawGnssMeasStage::markRelock). False when none.
    bool takeRelock(ShadowRelock& out);
    // external_build: the GNSS task's counts (the console, the pad status).
    void onMeasStats(const ShadowMeasStats& m);
    const ShadowMeasStats& measStats() const { return meas_st_; }
    // One flight-filter IMU tick; time64_us is its time widened to 64 bits
    // (unwrap32 against the clock it was stamped with).
    void onImu(const ShadowImu& s, int64_t time64_us);
    // A RAW_GNSS_EKF_MSG record, every log_period_s of IMU time while it runs.
    bool takeLog(RawGnssEkfData& out);
    // Its RAW_GNSS_ATT_MSG companion (attitude 1-sigma, IMU biases), filled at
    // the same tick: take it after takeLog.
    bool takeAttLog(RawGnssAttData& out);
    // The pad context (PAD_CONTEXT_MSG), for a replay of the flight: taken at
    // the IMU tick the flight computer first calls INFLIGHT (once a flight: a
    // restored flight, already in the air, takes none) -- the filter's state
    // and covariance, the ionosphere, the pad survey -- and written from
    // pad_context_delay_s later, one record a call (PCX_NAV, PCX_COV's parts,
    // PCX_SURVEY's, then a PCX_EPH for every ephemeris held): the payload in
    // out, len bytes, stamped now_us (FC micros). False when there is nothing
    // (yet) to write.
    bool takePadContext(uint8_t* out, size_t cap, size_t& len, uint32_t now_us);
    const EphemerisFrames& ephemerisFrames() const { return eph_frames_; }
    // A GNSS_CHECK_MSG record, one per receiver fix judged.
    bool takeCheckLog(GnssCheckData& out);
    // A HEADING_FIT_MSG record, one per epoch the heading fit took.
    bool takeHeadingLog(HeadingFitData& out);
    const HeadingFit& headingFit() const { return fit_; }

    bool running() const { return running_; }
    const Stats& stats() const { return st_; }
    const RawGnssEKF<float>& filter() const { return ekf_; }
    // The shadow's own stage (not with external_build).
    const EphemerisStore& ephemerides() const { return stage_.ephemerides(); }
    const RawMeasBuilder& builder() const { return stage_.builder(); }
    int ephemerisSatellites() const
    {
        return cfg_.external_build ? meas_st_.eph_sats : stage_.ephemerisSatellites();
    }
    const PadSurvey& survey() const { return survey_; }
    const FixCheck& fixCheck() const { return fixcheck_; }
    const SatJumpCheck& satCheck() const { return satcheck_; }
    // The last epoch's measurements as the filter took them (pad corrections
    // applied when they were in use), n of them: the console's per-satellite line.
    // The last epoch's measurements (none while the next one is being built
    // over them).
    const RawMeas* lastMeasurements(int& n) const { n = job_.active ? 0 : st_.last_meas; return built_.meas; }
    // What takes the ionosphere out now.
    uint8_t ionoSource() const;
    bool haveIono() const { return have_iono_; }
    const ShadowIono& iono() const { return iono_in_; }
    // Days from the parameters' file to the receiver's date (negative if the
    // file is dated later); false until both are known.
    bool ionoAgeDays(int& days) const;
    // The pad calibration's view of it (ShadowLink's pad snapshot).
    void padStatus(ShadowPadStatus& out) const;

    // The 64-bit time nearest ref64 whose low 32 bits are t32.
    static int64_t unwrap32(uint32_t t32, int64_t ref64)
    {
        return ref64 + static_cast<int32_t>(t32 - static_cast<uint32_t>(ref64));
    }

private:
    void start(double t);
    void stop(uint8_t why);
    void resync(double t);
    void applyDueEpochs(double t, uint8_t imu_flags);
    void startJob(double te, uint8_t imu_flags);
    void workJob(double t);
    void jobUnit(double t);
    void jobChecks(double age, double t);
    void jobScreen(double age);
    void jobScreenDry();
    void screenVerdict(bool drop, bool dry);
    void closeJob(double age, double t);
    void relock(int sys, int prn, double tow);
    void applyDueFixes(double t);
    void judge(const TimedFix& f, double t_fix);
    void dropFront();
    void fillLog(uint32_t time_us, uint8_t imu_flags);
    void stepHeading(double t, const float acc[3], const float gyro[3], float span, uint8_t imu_flags);
    bool jobHeading(double age);
    void jobHeadingFit();
    void handOverHeading(const HeadingFit::Result& r, double filt_minus_fit);
    bool headingsApart(const HeadingFit::Result& r, double filt_minus_fit) const;
    void capturePadContext(uint32_t tick_us);
    int64_t now() const { return clock_ ? clock_() : 0; }

    Config        cfg_;
    Clock         clock_ = nullptr;
    RawGnssEKF<float> ekf_;
    RawGnssMeasStage stage_;            // its own build (not with external_build)
    TimedRawEpoch pending_[2];          // raw epochs waiting for the IMU (built here)
    int           npending_ = 0;
    BuiltEpoch    pend_built_[2];       // built epochs waiting for it (external_build)
    int           pb_head_ = 0, npend_built_ = 0;
    // the epoch being worked (startJob .. closeJob), a slice a step
    enum : int { kJobBuild, kJobChecks, kJobHeading, kJobHeadingFit, kJobClock, kJobScreen, kJobScreenDry, kJobFuse,
                 kJobCarrier, kJobCarrierFuse, kJobClose };
    struct Job
    {
        bool          active = false;
        int           phase = kJobBuild;
        TimedRawEpoch e;                  // built here: the epoch
        double        te = 0.0;           // its instant on the filter's clock
        bool          in_flight = false;
        double        g_peak = 0.0;       // the largest |specific force| / g - 1 since the last epoch
        double        rot_sigma = 0.0;    // the turning body's allowance on its Dopplers (rot_lever_*), m/s
        int64_t       pr_k = 0;           // pr_period_s: the period of GPS time it falls in
        bool          screen = false;     // a boost epoch the boost screen judges (kJobScreen)
        bool          vcheck = false;     // the vertical check judges it if it can fit it (kJobScreen)
        int           dry_k = 0;          //   the scratch copy's next measurement (kJobScreenDry)
        double        dry_age = 0.0;      //   the epoch's age when it was copied
        RawGnssStats  dry_rs;             //   its counts
        int           fused = 0;          // measurements fused
        bool          cr_screened = false;  // its carrier rates screened (fuse mode's own unit)
        RawGnssStats  rs;
        int64_t       t0_us = 0;
        uint32_t      build_us = 0, update_us = 0, slices = 0;
        // the heading fit's epoch, between its two units
        double        hdg_v[3] = {};
        double        hdg_var = 0.0;
        double        hdg_t = 0.0;
        int           hdg_sats = 0;
    };
    Job           job_;
    BuiltEpoch    built_;               // its measurements: built in the job, or handed over
    RawGnssEKF<float> dry_;             // the boost screen's scratch copy of the filter (kJobScreenDry)
    static constexpr int kRelockQ = 16;  // jumped satellites waiting for the GNSS task (external_build)
    ShadowRelock  relock_[kRelockQ] = {};
    int           nrelock_ = 0;
    ShadowMeasStats meas_st_ = {};      // the GNSS task's counts (external_build)
    uint8_t       imu_flags_ = 0;       // the last IMU tick's
    ShadowSeed    seed_ = {};
    bool          have_seed_ = false;
    bool          running_ = false;
    bool          own_fix_ = false;     // a raw epoch has been applied since the start
    double        t_prev_ = 0.0;        // the last step
    double        t_tick_ = 0.0;        // the last IMU tick
    double        acc_sum_[3] = {};     // the ticks since the last step, time-weighted
    double        gyro_sum_[3] = {};
    double        last_resync_t_ = 0.0;
    double        next_log_t_ = 0.0;
    PadSurvey     survey_;
    SatJumpCheck  satcheck_;
    bool          jumped_[RawEpoch::kMaxObs] = {};
    FixCheck      fixcheck_;
    LevelGate     level_gate_;
    bool          levelled_ = false;    // the last step levelled (RGE_LEVEL)
    ShadowBaro    baro_ = {};           // the latest barometer sample, not yet applied
    bool          have_baro_ = false;
    double        baro_offset_ = 0.0;   // the filter's height less the barometer's
    double        baro_learn_s_ = 0.0;  //   learned over this long
    bool          baro_frozen_ = false; //   frozen at launch
    double        baro_launch_t_ = 0.0; //   then, the filter time
    bool          baro_seen_ = false;   //   a sample has gone into it
    double        last_baro_t_ = -1.0;  // the filter time the last sample was applied
    double        next_zupt_t_ = 0.0;   // landed: the next zero-velocity update is due
    ShadowMag     mag_ = {};            // the latest magnetometer sample, not yet applied
    bool          have_mag_ = false;
    uint32_t      mag_fused_run_ = 0;   // magnetometer headings fused since the start (on the ground)
    double        last_mag_fused_t_ = -1.0e9;   //   the last, filter time
    bool          restored_ = false;    // started in flight from a restored flight
    uint8_t       start_gen_ = 0;       // the seed's start_gen it started from
    double        last_aided_t_ = -1.0e9;   // raw measurements last applied
    double        last_baro_fused_t_ = -1.0e9;
    void          applyMag(double t, const float acc[3]);
    void          applyBaro(double t, uint8_t imu_flags);
    // the heading from thrust
    HeadingFit    fit_;
    bool          fit_ground_ = true;   // the last step was on the ground
    bool          fit_flight_ = false;  // launched with a fresh seed: epochs are taken
    double        fit_launch_t_ = 0.0;
    bool          handed_ = false;      // the heading handed over this flight
    bool          gate_full_ = false;   // act: the whole attitude's gate is open this flight
    double        boost_seen_t_ = -1.0e9;   // the filter's time the IMU last read past boost_screen_g
    // rr_dyn_window_s: the epochs within the window whose g excess could
    // still be its largest -- each larger than every one after it, so the
    // oldest is the window's peak (holdGExcess)
    struct GHeld
    {
        double        te;                 // the epoch's instant on the filter's clock
        double        g;                  // its g excess
    };
    static constexpr int kGHeld = 16;   // 1 s at 10 Hz is 11
    GHeld         g_held_[kGHeld] = {};
    int           n_g_held_ = 0;
    double        holdGExcess(double te, double g);
    int64_t       pr_period_k_ = -1;    // pr_period_s: the period the code was last fused in (< 0: none since the start)
    double        fit_last_te_ = -1.0;  // heading_fit_period_s: the last epoch the fit took (< 0: none this flight)
    bool          vert_latch_ = false;  // a vertical drop, and no epoch back within vert_release_mps since
    double        att_hold_until_ = -1.0e9;   // att_hold_s: epochs up to this instant are held (filter's clock)
    bool          rotOn() const
    {
        return cfg_.rot_lever_axial_m > 0.0 || cfg_.rot_lever_radial_m > 0.0 || cfg_.att_hold_rate_rps > 0.0;
    }
    bool          launch_seen_ = false; // a launch edge since the start (a restored flight has none)
    void          setFlightGate(bool full);
    void          setGroundGate();
    float         pad_ab_[3] = {}, pad_wb_[3] = {};   // the IMU biases at the seed
    double        pad_ll_[2] = {};      // its latitude and longitude: the fit's NED is the pad's
    float         pad_Pb_[6][6] = {};
    static constexpr int kHdgQ = 4;     // fit records waiting to be taken
    HeadingFitData hdg_[kHdgQ] = {};
    int           nhdg_ = 0;
    static constexpr int kFixQ = 4;      // fixes waiting for the IMU to pass them (0.4 s at 10 Hz)
    TimedFix      fixq_[kFixQ] = {};
    int           nfix_ = 0;
    GnssCheckData chk_[kFixQ] = {};      // check records waiting to be taken
    int           nchk_ = 0;
    ShadowIono    iono_in_ = {};        // the pad context's (the stage chooses with its own)
    bool          have_iono_ = false;
    bool          use_broadcast_ = false;   // the last epoch's ionosphere: the broadcast parameters
    bool          last_timed_ = false;
    uint8_t       pend_flags_ = 0;      // RGE_STALE / RGE_RESYNC / RGE_CLOCK_STEP since the last record
    RawGnssEkfData log_ = {};
    bool          log_ready_ = false;
    RawGnssAttData att_log_ = {};
    bool          att_log_ready_ = false;
    // the pad context (takePadContext)
    EphemerisFrames eph_frames_;
    bool          pcx_armed_ = false;    // a ground tick since the last launch edge
    bool          pcx_pending_ = false;  // taken, not all written
    uint32_t      pcx_due_us_ = 0;       // written from then (FC micros)
    int           pcx_stage_ = 0;        // the next record
    int           pcx_eph_next_ = 0;
    int           pcx_neph_ = 0;         // the ephemerides held at the launch tick: the later are in the flight's log
    PadContextNav pcx_nav_ = {};
    float         pcx_cov_[PCX_COV_VALUES] = {};
    static constexpr int kPcxSurveyMax = 96;
    PadSurveyEntry pcx_survey_[kPcxSurveyMax] = {};
    int           pcx_nsurvey_ = 0;
    PadSurvey::Summary pcx_sum_ = {};
    uint64_t      prop_sum_ = 0;        // IMU steps since the last record
    uint32_t      prop_n_ = 0;
    Stats         st_;
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_SHADOW_H
