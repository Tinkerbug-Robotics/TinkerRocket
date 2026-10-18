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
// The Doppler change check (dv_check; off by default: DopplerChange.h). A
// loop still settling after the thrust, or a receiver handing over an old
// velocity, can sit inside the innovation gate, which judges each Doppler by
// the filter's velocity -- one the stale Dopplers have already pulled. How
// each Doppler CHANGED since the epoch before cannot hide so: the IMU says
// how the receiver's velocity changed, and each range rate must have changed
// by that along its line of sight, plus the satellite's own change and the
// clock drift's, which every satellite shares. The filter's velocity is at
// both ends and drops out, so a filter already wrong does not judge the data
// by itself (the boost screen's weakness). Each epoch, before anything of it
// is fused (its first unit, before the stress term goes on the Dopplers):
//   - a satellite whose change is more than dv_sat_k of its 1-sigmas off the
//     epoch's own fit -- one velocity change and one clock rate fitted to
//     every satellite, so an attitude error, an antenna off the IMU turning
//     with the body or the IMU's scale, each one change for the receiver as a
//     whole, is taken out -- fails, and its Doppler is held out of the update
//     until dv_hold_s after its last failure;
//   - the epoch fails if that fitted change is more than dv_epoch_k sigmas
//     off the IMU's, its allowances in: the IMU's specific-force change
//     turned by dv_att_allow_deg (so an attitude error, which only turns it,
//     leaves the part along it to judge: a lagging loop reports less of the
//     change, along it), off by dv_scale_allow of itself, an accelerometer
//     bias of dv_bias_allow_mps2 over the interval, and dv_lever_allow_m of
//     antenna offset at the gyro's change of rate and turn; or if more than
//     half its satellites fail. Then every Doppler is held out until
//     dv_epoch_hold_s after the last failed epoch.
// With dv_noise_tau_s, the epoch's changes also measure the Dopplers' own
// noise: their residuals' spread off the fit (a robust one, the median's)
// past what the model gives them, held at its peak and let fall with that
// time constant, goes on every Doppler's variance (acting, in flight) and on
// the changes' own -- a loop settling after the thrust is noisy on every
// satellite at once, and holding all but the few that pass by luck fuses
// those few at a sigma the loop is not keeping. Holding out more than
// dv_hold_all_frac of an epoch's Dopplers holds out all of them: what is
// left is the luck's, on a thin geometry.
// With dv_window_s, each satellite's residual and the epoch's change are
// also summed over that long (leaky) and judged so: a slow settling moves a
// Doppler by a few cm/s an epoch, under one epoch's noise, but by tens of
// cm/s a second. The window's lever-arm allowance is capped at the arm times
// the body's rate now and its largest in the window: the antenna's velocity
// changes telescope like the Dopplers', so summed epoch by epoch a
// weathercocking coast's turns would add up to a window that misses a slow
// ramp. With dv_latch_k, a failed epoch holds every Doppler not for
// dv_epoch_hold_s but until the epoch's Dopplers as they stand agree with the
// filter's velocity -- the IMU's, the barometer's and the pseudoranges' while
// they were held -- within dv_latch_k sigmas of the filter's velocity
// covariance, the fit's and dv_floor_mps. No lever-arm allowance there: an
// antenna off the IMU only keeps the latch on while the body turns, where the
// allowance would let a bias of that size back in at a drogue's deployment. A
// receiver whose velocity ramped away and stays there (a coherent bias the
// window saw while it grew) then never comes back as a step the update would
// take partly into the attitude, and one that settles back to the truth comes
// back when it has. dv_deweight grows the failed Doppler's sigma by its excess
// instead of holding it out. Observe counts and holds nothing. It acts in
// flight only (dv_ground: on the pad too, where zero velocity holds the
// velocity anyway); held-out Dopplers count as neither fused nor refused, so
// the update's refusal persistence (reject_persist_rr) does not see them.
// With dv_hdg_unset_deg, a flight whose launch had no magnetometer heading
// (none fused within heading_mag_recent_s of leaving the ground, as act's
// launch prior judges it; a shadow that is not the navigation filter, and a
// flight restored in the air, never know) also allows the IMU's change
// turned that far about the vertical (DopplerChange.h headingChord): the
// heading is then the start's guess, any distance off. Without it, Grid B's
// 16 no-fault flights with the pad's heading 120-180 deg off held Dopplers
// before apogee in 4-10 of them, latched to landing in up to 3, and left vz
// at the true apogee up to 4.6 m/s off where the filter without the check
// had 0.3 (one flight at 180 deg: 4.1). With 180, up to 160 deg off they fly
// as without the check (one epoch held before apogee in one flight); from
// 170 deg one or two of the 16 still hold the coast's Dopplers: a heading
// that far wrong pulls the filter's own tilt and accelerometer bias off in
// the boost (no small correction turns a heading through 180 deg), so the
// IMU's change is wrong along the thrust too. The vertical part, where a
// loop's lag and a stale receiver show in a boost near vertical, is judged
// as before.
// A flight restored in the air starts the check over: no reference, no
// window, no latch. A receiver the latch was holding out for a bias that
// stands (it no longer changes) is then fused for the rest of the flight, as
// without the check. Carrying the latch through the snapshot would not keep
// it out: the restore declares the velocity restore_vel_sigma_mps unsure (20
// m/s on the mosaic image), and the latch's level test, which takes the
// filter's velocity covariance, would let a 12 m/s bias go at its first test.
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
// (Grid B, 2026-10-17: up to 42 deg in the snatch's first 3 s, no fault).
// Three allowances, each off at 0, need only a bound on r or the gyro:
//   - each Doppler's sigma takes, in quadrature, the most omega x r can be
//     (rotLeverSigma): rot_lever_axial_m (along the nose) times the rate
//     across the nose, plus rot_lever_radial_m (across it) times the whole
//     rate -- an antenna on the axis does not move as the body rolls, one off
//     it does. The rate is the gyro less its bias at its largest since the
//     epoch before, so the epoch's own instant is in it. The boost screen,
//     the vertical check and the update all judge the Dopplers with it. With
//     rot_lever_min_rps, only on an epoch whose rate across the nose reached
//     that: the snatch and a tumble, not a boost's weathercocking or the
//     pitch-over at apogee, where the votes and the raw vote guard read the
//     filter's vertical velocity;
//   - an epoch whose rate across the nose reached att_hold_rate_rps, and each
//     within att_hold_s after one, keeps GNSS off the attitude and the gyro
//     bias (the unaided twin's mask, for those epochs): the snatch and a
//     tumble. Roll alone never holds it: a spinning rocket keeps its GNSS
//     attitude;
//   - an epoch whose rate across the nose reached rot_skip_rate_rps, and each
//     within rot_skip_s after one, fuses nothing: it is closed as the
//     vertical check closes one it drops (the clock's step stands). The hold
//     lets the velocity take those epochs, and with them the error a wrong
//     heading built into it through the snatch's large specific force -- the
//     heading's best evidence of the flight, then never read. Skipped, that
//     error is still in the velocity when the first epoch after the turn
//     comes, and the update reads it through the covariance the turn built.
// With rot_after_apogee all three act only once the flight computer has
// called apogee: the drogue's snatch and the descent. The boost and the
// coast -- where a boost that weathercocks would be held, and where the votes
// and the raw vote guard read the filter -- fly as without them.
//
// The antenna's offset where the rocket's owner measured it (an optional
// per-rocket setting, unknown by default: then everything above is as it
// was, bit for bit). Known (the EKF's lever_on: applyAntennaOffset at the
// start, setAntenna on the ground), every Doppler and pseudorange is
// predicted at the antenna -- C (omega x r) and C r, each at its own instant
// (TR_RawGnssEKF.h) -- and the bounds above give way to what the measured
// offset could be off by: each Doppler's sigma takes rotLeverSigma with
// antenna_sigma_axial_m and antenna_sigma_radial_m in place of
// rot_lever_axial_m and rot_lever_radial_m, from antenna_min_rps across the
// nose in place of rot_lever_min_rps, and the Doppler change check's lever
// allowance (whose changes are now the antenna's) the larger of the two
// sigmas in place of dv_lever_allow_m -- plus what the attitude's error does
// to the lever's change both ends predicted (DopplerChange.h knownLeverArm):
// with the heading known, the attitude allowance's chord of the offset; with
// it unset (dv_hdg_unset_deg's judgment), any heading turns the predicted
// change, up to twice its size (0.3 m on a coast weathercocking with the
// heading 60 deg off: a chi-square of 19.9 against the 4-sigma test's 16,
// unknown's 0.89), so there the prediction is taken back out of the fitted
// change and the whole offset allowed, its size plus the larger sigma; and
// out of the latch's level (dv_latch_k), which would otherwise hold up to
// twice the lever's velocity where unknown's holds it once, and the latch
// longer than unknown's. The Dopplers the latch then lets go to the update
// are still predicted at that heading, so with it unset each Doppler's sigma
// also takes what any heading does to the predicted lever's velocity
// (leverUnsetM: rotLeverSigma of twice the offset's part along the nose and
// twice its part across), from rot_lever_min_rps across the nose, where the
// bounds start: below it unknown's own filter takes the lever's whole
// velocity unallowed. Without it, 0.3 m in a 5 rad/s swing just after a
// latched epoch, the heading 180 deg off, took the velocity 5.5 m/s off, and
// the latch held the good Dopplers out against it (unknown's: 0.31 m/s).
// Grid B with no magnetometer: the ascent within 0.1 deg of without it (one
// coast 0.55 deg worse, to unknown's), the descent's p90 worse by over
// 0.5 deg in 42 of 384 runs (79 at 0.3 m), the price of keeping a swing's
// mis-predicted Dopplers out (from any rate: 70 and 109, and the vertical
// velocity at apogee moved too). The hold and the skip are untouched. The
// offset is a prior, never the truth: a phase centre a few cm from where the
// owner measured is what the sigmas are for. setAntenna in flight waits for
// the ground.
//
// The lever roll (the EKF's lever_att_mode observe; the change check on). A
// turning antenna's velocity turns with the body, so it says how the body is
// rolled about the nose -- which nothing else on a vertical thrust does. In
// one epoch a roll error delta looks just like a velocity error (the antenna's
// velocity turned by delta about the nose, the same along every line of
// sight), so it is read where the IMU ties the velocity: each checked epoch's
// fitted change w (the antenna's velocity change less the IMU's and the
// lever's, predicted at the filter's attitude) against g, the change a radian
// of roll would make -- n2 x v2 - n1 x v1 for the lever's velocity v and the
// nose n at the two ends, plus n x dv_sf for the IMU's specific-force change
// (none for a thrust along the nose). The epoch's roll correction is g' C^-1 w
// / g' C^-1 g (C the fit's covariance of w), its 1-sigma 1 / sqrt(g' C^-1 g);
// the sums since the start give the run's. Counted and logged, nothing fused.
//
// The spinning body (all off by default; the windup study, 2026-10-18). An
// antenna turning about its boresight winds its carrier: -k lambda p / 2pi on
// every Doppler at a roll rate p (RawGnssWindup.h), 0.19 m/s per rev/s on
// L1/E1 and 1.7 mm/s per rev/s more on BeiDou B1I. The clock drift's walk
// (the mosaic's 100 m^2/s^3) takes the common part; nothing takes the
// difference between wavelengths, or the part a rate across the boresight
// gives each satellite. With windup_k, each epoch just built has it taken
// off every Doppler (and carrier rate) before anything judges it -- the
// change check, the satellite checks, the heading fit, the vertical check,
// the boost screen, the update: the gyro's rate (less its bias) about the
// nose at the Doppler's own instant (the epoch's less rr_lag_s, read from the
// last 0.64 s of steps; an epoch older than that is left as it came and
// counted), times each system's wavelength (the EKF's cr_carrier_hz). With
// windup_geom, Wu's in full from each satellite's line of sight in the body
// (the filter's attitude). No antenna offset is needed: only the boresight's
// direction, the nose. With windup_monitor (observe), each epoch's Dopplers at
// the prior are fitted for the roll rate their wind-up differs by between
// wavelengths (RawGnssWindup.h's interband fit) and set against the gyro's.
// With dv_lever_mode 1, the change check's lever allowance takes the nose's
// own turn (a roll does not move an antenna on the nose's axis) and a radial
// bound, not the whole rate.
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

#include <cmath>
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
#include "RawGnssWindup.h"
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
        // Where the EKF's hdg_obs_mode (GNSS's heading gain by how well the
        // moment observes it; off by default) applies (HeadingObsScope):
        // everywhere, the pad too -- where nothing horizontal pushes, so GNSS
        // stops moving the gyro bias about the vertical -- or in flight only,
        // or from launch to the flight computer's apogee call (the drogue's
        // descent then fully open, as today).
        uint8_t  heading_obs_scope = 0;
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
        // the Doppler change check (above; DvCheck, off by default)
        uint8_t  dv_check = 0;
        double   dv_sat_k = 5.0;                 // a satellite this many 1-sigmas off the epoch's fit fails
        double   dv_epoch_k = 5.0;               // an epoch whose change is this many sigmas off the IMU's fails
                                                 //   (the root of a 3-dof chi-square, the allowances in)
        double   dv_floor_mps = 0.03;            // each change's 1-sigma: its two Dopplers', this,
        double   dv_sigma_per_g = 0.1;           //   and this much per g of the IMU's peak excess since the epoch
                                                 //   before (a loop under stress), in quadrature
        double   dv_att_allow_deg = 30.0;        // the epoch test's allowances: the IMU's change turned this much,
        double   dv_scale_allow = 0.01;          //   its specific force's part off by this fraction,
        double   dv_bias_allow_mps2 = 0.1;       //   an accelerometer bias this size,
        double   dv_lever_allow_m = 0.3;         //   an antenna this far from the IMU, turning with the body
        double   dv_hold_s = 1.0;                // a failed satellite held out until this long after its last failure
        double   dv_epoch_hold_s = 0.5;          // a failed epoch: every Doppler held out until this long after
        double   dv_window_s = 0.0;              // also judged summed over this long, leaky (0: epoch by epoch only)
        bool     dv_deweight = false;            // grow a failed Doppler's sigma by its excess, not hold it out
        bool     dv_ground = false;              // act on the ground too (else in flight only)
        double   dv_noise_tau_s = 0.0;           // the Dopplers' measured excess noise on their variance, let fall
                                                 //   with this time constant (0: not measured)
        double   dv_hold_all_frac = 0.5;         // more than this share of an epoch's Dopplers held: all of them
                                                 //   (1: never)
        double   dv_latch_k = 0.0;               // a failed epoch holds every Doppler until the epoch's Dopplers
                                                 //   agree with the filter's velocity within this many sigmas
                                                 //   (0: off, dv_epoch_hold_s instead)
        double   dv_hdg_unset_deg = 0.0;         // a launch with no magnetometer heading: the epoch test also
                                                 //   allows the IMU's change turned this far about the vertical
                                                 //   (180: any heading; 0: dv_att_allow_deg alone)
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
        double   rot_lever_min_rps = 0.0;     //   on an epoch whose rate across the nose reached this, rad/s
        double   att_hold_rate_rps = 0.0;     // an epoch whose rate across the nose reached this, rad/s, and
        double   att_hold_s = 0.0;            //   every epoch within this long after one: GNSS held off the
                                              //   attitude and the gyro bias (RawGnssEKF::setAttHold)
        double   rot_skip_rate_rps = 0.0;     // an epoch whose rate across the nose reached this, rad/s, and
        double   rot_skip_s = 0.0;            //   every epoch within this long after one: nothing fused (dropped)
        bool     rot_after_apogee = false;    // all three only after the flight computer's apogee call
        // The antenna's offset, known (the EKF's lever_on, above): what it
        // could be off by. The offset itself is the EKF's lever_b_m.
        double   antenna_sigma_axial_m = 0.05;   // along the nose, 1-sigma, m
        double   antenna_sigma_radial_m = 0.02;  //   and across it
        double   antenna_min_rps = 0.0;          // on the Dopplers from this rate across the nose, rad/s (0: any)
        uint8_t  antenna_att_mode = 1;           // the EKF's lever_att_mode when known (LeverAttMode: observe)
        // The carrier wind-up (above: the spinning body; RawGnssWindup.h). 0
        // or false: off, the Beetle's filter and the V9 mosaic image as flown.
        double   windup_k = 0.0;              // cycles a turn about the boresight (the nose) each Doppler and
                                              //   carrier rate carries, taken off from the gyro (0: off; the
                                              //   Goblin's helix 0.99, spin rig 10/16)
        bool     windup_geom = false;         //   and the part the rate across the boresight gives each
                                              //   satellite (Wu's in full, from its line of sight)
        bool     windup_monitor = false;      // observe: the roll rate the Dopplers' wind-up implies against
                                              //   the gyro (RawGnssWindup.h's interband fit, each epoch)
        double   windup_monitor_k = 0.99;     //   its k
        double   windup_monitor_tau_s = 1.0;  //   its leaky mean's time constant
        // The Doppler change check's lever-arm allowance (dv_lever_allow_m):
        // 0, the body's whole rate (dv_lever_allow_m (|dw| + |w|^2 dt)); 1,
        // an antenna dv_lever_allow_m along the nose turns only with the nose
        // -- the change of the nose's own rate in NED, which a roll about it
        // leaves alone -- and one dv_lever_radial_m across it with the whole
        // rate (its turn capped at twice its speed). At 1 rev/s of roll, 0
        // allows 1.4 m/s an epoch and a slow coherent ramp goes through (the
        // windup study: 0 of 192 caught from 1 rev/s on). The radial bound
        // costs the same way: 1 cm lost them from 5 rev/s, 3 cm from 3. None
        // (0, the default) caught every one from 0 to 20 rev/s; with the
        // antenna truly 1 cm off the roll axis it held Dopplers in 1 run of
        // 48 at 9.5 rev/s, at no attitude cost.
        uint8_t  dv_lever_mode = 0;
        double   dv_lever_radial_m = 0.0;
    };

    // The Doppler change check (Config::dv_check).
    enum DvCheck : uint8_t
    {
        kDvOff = 0,
        kDvObserve = 1,        // checked and counted, nothing held
        kDvAct = 2,            // and the failed Dopplers held out (or de-weighted)
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
    // Where the EKF's hdg_obs_mode applies (Config::heading_obs_scope)
    enum HeadingObsScope : uint8_t
    {
        kObsEverywhere = 0,
        kObsFlight = 1,        // from launch until back on the ground
        kObsAscent = 2,        // from launch to the apogee call (kShadowApogee)
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
        // the Doppler change check
        uint32_t dv_checks = 0;            // epochs checked: a reference, the IMU's history across, five changes
        uint32_t dv_sat_fails = 0;         //   satellites that failed (epoch by epoch, or summed)
        uint32_t dv_sat_win_fails = 0;     //     of them, on the window's sum alone
        uint32_t dv_epoch_fails = 0;       //   epochs that failed (their change, or over half their satellites)
        uint32_t dv_epoch_win_fails = 0;   //     of them, on the window's sum alone
        uint32_t dv_held_rr = 0;           // Dopplers held out (or de-weighted): acting
        uint32_t dv_held_epochs = 0;       //   epochs with any
        float    dv_last_chi2 = 0.0f;      // the last epoch checked: its chi-square (3 dof)
        float    dv_last_along = 0.0f;     //   its change less the IMU's, along the specific force's, m/s
        float    dv_last_chi2_win = 0.0f;  //   the window's chi-square (dv_window_s)
        float    dv_max_chi2 = 0.0f;       // the largest epoch chi-square seen
        float    dv_max_e = 0.0f;          // the largest satellite residual seen, 1-sigmas
        float    dv_noise_mps = 0.0f;      // the Dopplers' measured excess noise (dv_noise_tau_s), 1-sigma, now
        float    dv_noise_max_mps = 0.0f;  //   and its largest
        uint32_t dv_all_held = 0;          // epochs whose Dopplers were all held for the share held (dv_hold_all_frac)
        uint32_t dv_latches = 0;           // dv_latch_k: latches begun (a failed epoch, none on)
        uint32_t dv_latch_releases = 0;    //   and let go (the Dopplers back with the filter's velocity)
        uint32_t dv_latch_epochs = 0;      //   epochs checked while one was on
        float    dv_last_level_chi2 = 0.0f;//   the last level test's chi-square (3 dof; -1: no fit)
        uint8_t  dv_hdg_unset = 0;         // the check takes dv_hdg_unset_deg now: no magnetometer heading (on
                                           //   the pad, now; in flight and landed, at the launch)
        // the turning body (rot_lever_*, att_hold_*, rot_skip_*)
        float    rot_last_sigma_mps = 0.0f;   // the last epoch's rotation allowance on its Dopplers
        float    rot_max_sigma_mps = 0.0f;    //   the largest any epoch took
        float    rot_last_across_rps = 0.0f;  // the last epoch's largest rate across the nose
        uint32_t att_held = 0;             // epochs whose GNSS was held off the attitude (att_hold_rate_rps)
        bool     att_last_held = false;    //   the last one's
        uint32_t rot_skipped = 0;          // epochs that fused nothing while the body turned (rot_skip_rate_rps)
        bool     rot_last_skipped = false; //   the last one's
        // the antenna's offset (setAntenna)
        uint32_t antenna_sets = 0;         // offsets taken (known or unknown)
        uint8_t  antenna_pending = 0;      // one waits for the ground
        // its roll terms observed (the EKF's lever_att_mode observe, with the
        // change check): each checked epoch's fitted change, w, against the
        // roll a turning antenna points to (lever roll, below)
        uint32_t lever_roll_epochs = 0;    // epochs fitted for it
        float    lever_roll_last_deg = 0.0f;       // the last one's roll correction, deg (0: no turn to see it)
        float    lever_roll_last_sigma_deg = 0.0f; //   its 1-sigma, deg (0: none)
        double   lever_roll_info = 0.0;    // summed since the start: g' C^-1 g, 1/rad^2
        double   lever_roll_hy = 0.0;      //   and g' C^-1 w, 1/rad: their ratio the roll the whole run points to
        // the carrier wind-up (windup_k)
        uint32_t windup_epochs = 0;        // epochs whose Dopplers (and carrier rates) were corrected
        uint32_t windup_misses = 0;        //   whose Doppler instant the gyro's history no longer held: left as is
        uint32_t windup_cr_misses = 0;     //   carrier rates whose interval it did not hold: dropped
        float    windup_last_p_rps = 0.0f; // the last epoch's: the gyro's roll rate at its Doppler instant, rad/s
        float    windup_last_l1_mps = 0.0f;//   the correction an L1/E1 Doppler took, m/s
        float    windup_max_l1_mps = 0.0f; // the largest of those
        // the wind-up monitor (windup_monitor)
        uint32_t wumon_epochs = 0;         // epochs it fitted (two wavelengths, six Dopplers or more)
        float    wumon_last_p_rps = 0.0f;  //   the last: the roll rate its Dopplers' wind-up implies, rad/s
        float    wumon_last_sig_rps = 0.0f;//     its 1-sigma
        float    wumon_last_gyro_rps = 0.0f;//    the gyro's then
        float    wumon_dp_rps = 0.0f;      // implied less the gyro's, leaky mean (windup_monitor_tau_s), rad/s
        float    wumon_dp_sig_rps = 0.0f;  //   its 1-sigma
        float    wumon_max_z = 0.0f;       // the largest |that| / its 1-sigma, once 10 epochs are in
        // the change check's clock and lever, the last epoch checked
        float    dv_last_c_mps = 0.0f;     // its fitted change of the clock rate, m/s
        float    dv_last_wu_dc_mps = 0.0f; //   the L1 wind-up's change the gyro gives over the same interval
        float    dv_last_lever_mps = 0.0f; //   the lever allowance the epoch test took
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
    // The antenna's offset from the IMU (body FRD, m; known false: unknown,
    // the bounds), as the owner's setting has it now: taken at once on the
    // ground, at the first ground tick after a flight. The same as
    // applyAntennaOffset gives a start (ReceiverProfileApply.h).
    void setAntenna(bool known, const double b_m[3]);
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
    void jobDvCheck(double age);
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
    enum : int { kJobBuild, kJobDvCheck, kJobChecks, kJobHeading, kJobHeadingFit, kJobClock, kJobScreen,
                 kJobScreenDry, kJobFuse, kJobCarrier, kJobCarrierFuse, kJobClose };
    struct Job
    {
        bool          active = false;
        int           phase = kJobBuild;
        TimedRawEpoch e;                  // built here: the epoch
        double        te = 0.0;           // its instant on the filter's clock
        bool          in_flight = false;
        double        g_peak = 0.0;       // the largest |specific force| / g - 1 since the last epoch
        double        rot_sigma = 0.0;    // the turning body's allowance on its Dopplers (rot_lever_*), m/s
        bool          rot_skip = false;   //   or nothing of it fused (rot_skip_rate_rps)
        bool          rot_dropped = false;//   and its clock taken, so it closes as a dropped epoch
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
        // the carrier wind-up: the gyro's roll rate at the Doppler's instant (rad/s), and whether it had one
        double        wu_p = 0.0;
        bool          wu_ok = false;
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
    bool          obs_apogee_ = false;  // heading_obs_scope kObsAscent: the apogee call seen this flight
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
    // the Doppler change check (dv_check)
    DopplerRef    dv_ref_;              // the last epoch's Dopplers, the next change's start
    DopplerRef    dv_next_;             //   this epoch's, taken by the check
    dvc::Row      dv_rows_[RawEpoch::kMaxObs] = {};
    struct DvHold
    {
        uint8_t       sys, prn;
        double        until;            // held out to this epoch time (the filter's clock)
        float         excess;           // m/s: dv_deweight's
    };
    static constexpr int kDvHold = 24;
    DvHold        dv_hold_[kDvHold] = {};
    int           n_dv_hold_ = 0;
    double        dv_epoch_until_ = -1.0e9;   // every Doppler held out to this epoch time
    float         dv_epoch_excess_ = 0.0f;
    float         dv_w_now_[3] = {};    // the gyro's last step (rad/s, body), for the lever arm's allowance
    float         dv_w_ref_[3] = {};    //   at the last check
    float         dv_w_peak_ = 0.0f;    //   and its largest |rate| since
    // dv_window_s: the leaky sums -- each satellite's residual (m/s) and the
    // leaky mean of its change's variance, in dv_ref_'s order; the epoch's
    // change and the leaky mean of its covariance, the IMU's specific-force
    // change and the bias and lever allowances, summed
    double        dv_S_[RawEpoch::kMaxObs] = {}, dv_V_[RawEpoch::kMaxObs] = {};
    double        dv_S2_[RawEpoch::kMaxObs] = {}, dv_V2_[RawEpoch::kMaxObs] = {};
    double        dv_SW_[3] = {}, dv_CW_[3][3] = {}, dv_SF_[3] = {}, dv_SB_ = 0.0, dv_SL_ = 0.0;
    double        dv_wwin_ = 0.0;       //   the body's largest rate in the window (leaky), rad/s
    double        dv_noise_var_ = 0.0;  // dv_noise_tau_s: the Dopplers' excess variance, (m/s)^2
    double        dv_noise_te_ = 0.0;   //   the epoch it was last updated at
    bool          dv_latch_ = false;    // dv_latch_k: a failed epoch's latch, on
    bool          dv_hdg_known_ = false;// dv_hdg_unset_deg: the heading is the magnetometer's (on the ground,
                                        //   each step; in flight, as the launch left it)
    void          dvReset();
    void          leverRoll(double age, const double w[3], const double C[3][3], const DopplerChangeImu& imu);
    double        dvLevelChi2(double row_var, const float v_lev[3]);
    double        att_hold_until_ = -1.0e9;   // att_hold_s: epochs up to this instant are held (filter's clock)
    double        rot_skip_until_ = -1.0e9;   // rot_skip_s: epochs up to this instant fuse nothing (filter's clock)
    bool          rotOn() const
    {
        return cfg_.rot_lever_axial_m > 0.0 || cfg_.rot_lever_radial_m > 0.0 || cfg_.att_hold_rate_rps > 0.0
               || cfg_.rot_skip_rate_rps > 0.0 || cfg_.ekf.lever_on;
    }
    // the Dopplers' allowance for the antenna: the bounds, or with the offset known what it could be off by
    double        leverAxialM() const { return cfg_.ekf.lever_on ? cfg_.antenna_sigma_axial_m : cfg_.rot_lever_axial_m; }
    double        leverRadialM() const
    {
        return cfg_.ekf.lever_on ? cfg_.antenna_sigma_radial_m : cfg_.rot_lever_radial_m;
    }
    // and with the offset known and the heading unset (!dv_hdg_known_, as the latch's dvLevelChi2 and the
    // change's knownLeverArm take it), what any heading does to the lever's velocity each Doppler is still
    // predicted with: up to twice its size, so the offset's part along the nose (axial) or across it, twice over
    double        leverUnsetM(bool axial) const
    {
        const double* r = cfg_.ekf.lever_b_m;
        return 2.0 * (axial ? std::fabs(r[0]) : std::sqrt(r[1] * r[1] + r[2] * r[2]));
    }
    double        leverMinRps() const { return cfg_.ekf.lever_on ? cfg_.antenna_min_rps : cfg_.rot_lever_min_rps; }
    double        dvLeverM() const
    {
        return cfg_.ekf.lever_on ? std::fmax(cfg_.antenna_sigma_axial_m, cfg_.antenna_sigma_radial_m)
                                 : cfg_.dv_lever_allow_m;
    }
    // the Doppler change check's other end with the offset known (dv_ref_'s): the lever velocity its
    // Dopplers were predicted with and the nose then, NED (the lever roll's too); ok: dv_ref_ was taken
    // with the offset known
    float         lev_ref_v_[3] = {}, lev_ref_n_[3] = {};
    bool          lev_ref_ok_ = false;
    // setAntenna's, waiting for the ground
    bool          ant_pending_ = false;
    bool          ant_known_ = false;
    double        ant_b_m_[3] = {};
    void          applyAntenna();
    // the carrier wind-up (windup_k, windup_monitor): the gyro's steps (less
    // its bias, rad/s) to read a Doppler's own instant from, 0.64 s of them at
    // 100 steps a second -- an epoch is dropped 0.3 s old on the mosaic
    static constexpr int kWindupRing = 64;
    windup::RateRing<kWindupRing> wu_ring_;
    windup::Monitor wu_mon_;
    double        wu_ref_p_ = 0.0;      // the roll rate at the change check's reference epoch (rad/s)
    bool          wu_ref_ok_ = false;
    bool          windupRingOn() const { return cfg_.windup_k > 0.0 || cfg_.windup_monitor; }
    bool          dvJobOn() const { return cfg_.dv_check != kDvOff || cfg_.windup_monitor; }
    // the epoch just built (built_), age s old: its Dopplers less the
    // wind-up (windup_k), and the gyro's rate at its Doppler instant (job_.wu_p)
    void          applyWindup(double age);
    // windup_monitor: this epoch's Dopplers at the prior (dv_next_), fitted
    void          windupMonitor();
    // dv_lever_mode 1: the nose's rate in NED (R (w x x_body), rad/s) at the
    // last step and at the last check, its largest change a second since the
    // check, and the largest rate across the nose since it and in the window
    float         dv_nd_now_[3] = {}, dv_nd_ref_[3] = {};
    float         dv_ndd_peak_ = 0.0f;
    bool          dv_nd_have_ = false;
    float         dv_wp_peak_ = 0.0f;
    double        dv_wpwin_ = 0.0;
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
