// raw_replay.cpp -- the Beetle's raw-GNSS navigation filter, replayed offline
// from one OC flight log (#1600 attitude work).
//
// The flight computer's raw task (flight_computer/main/main.cpp, the raw-GNSS
// section) feeds RawGnssShadow: every EKF tick's IMU input with its flags,
// each barometer sample BaroGatePolicy accepts, each new magnetometer sample,
// once a second the seed, and the epochs its GNSS task builds
// (RawGnssMeasStage) from the LC86G's MSM7, placed on the MCU clock by the
// driver's time sync. This program does the same from extract.py's event
// stream, with the firmware's own sources compiled unmodified, and every bench
// flag of the image set at run time:
//
//   --heading-mode N      TR_HEADING_FIT: 0 off, 1 observe, 2 act
//   --pad-sigma-deg X     TR_BENCH_HEADING_PAD_SIGMA_DEG (act's heading 1-sigma from launch)
//   --handoff-deg X       TR_BENCH_HEADING_HANDOFF_DEG (-1: never handed over)
//   --gate-full 0|1       TR_BENCH_HEADING_GATE_FULL
//   --tilt-sigma-deg X    TR_BENCH_LAUNCH_TILT_SIGMA_DEG (0: off)
//   --carrier N           TR_BENCH_CARRIER_MODE: 0 off, 1 observe, 2 fuse
//   --half-cycle-repair N TR_BENCH_HALF_CYCLE_REPAIR
//   --mag-error-deg X     TR_BENCH_HEADING_ERROR_DEG: added to the declination
//   --recorded-mag-error-deg X   what the log's own image was built with
//
// The log keeps about a second of the pad before ignition, so the replay
// starts from the raw filter's own state at the log's first record (0x96 and
// 0x9B) with the covariance its pad context (0x9E) carried at the launch tick
// (its velocity block scaled to the first record's sigmas). A magnetometer
// heading error other than the recorded one turns that pad state about the
// vertical by the difference, as the magnetometer would have on the pad, and
// north (the declination the filter takes the magnetometer with) with it.
// See README.md for what is and is not reproduced.
//
// --digest FILE writes one line of hashes per filter step over a fixed, named
// list of what the step left (the navigation state, the records it would log,
// its counters, the filter's whole state), and --fw-defaults leaves the
// heading's settings at the firmware's defaults, as the flight image without
// bench flags has them: bitident.sh compares two firmware trees that way.
//
// The V8/V9 with the mosaic-G5 (--receiver mosaic): the same filter as the
// mosaic-only build's one navigation filter -- the mosaic's profile and the
// configuration startRawShadow() gives it there, the heading's settings at
// the firmware's defaults -- fed from the SBF blocks the OC logged (0x95)
// through the firmware's own decoders (mosaic_feed.h), started on the pad
// from nothing (--cold-start: no record of the filter's own in the log),
// with the day's broadcast ephemerides beside the log's own if asked
// (--rinex FILE). --unaided masks the GNSS rows' gain on the attitude and the
// gyro bias (the acceptance twin). And the apogee quorum the flight computer
// would call from it (flight_votes.h: TR_KinematicChecks, GnssAscentGate, the
// raw vote guard, the receiver's own fix or a sim's 'V' events as the GPS
// voter), on with the mosaic or with --quorum, reported in the events JSON.
//
// A simulated flight (sim_events.py: Grid B) comes as 'X' events instead of
// SBF: each epoch's satellites already placed by the sim's receiver model,
// taken through the GNSS task's stage -- seen, its lock ages, the cap, the
// builder's floors, mask and noise model (sim_epoch.h) -- and handed to the
// filter as a built epoch. Its faults are in the events; the decoded-epoch
// faults (--cut-*, --corrupt-at-s, --relock-min-lock-ms) are refused with it.
// With --truth-nav (the sim's truth, truth_nav.h) a second quorum runs on a
// perfect filter beside the first, report only: the truth's own call.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <new>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <RocketComputerTypes.h>

#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "Rtcm3Msm.h"
#include "baro_gate_policy.h"

// The replay restores the filter's internal state (its pad state and
// covariance, the builder's lock history) that no public call sets: the
// firmware's headers, read with their private members open. The .cpp files
// are compiled as they are; access does not change a class's layout.
#define private public
#define protected public
#include "RawGnssMeasStage.h"
#include "RawGnssShadow.h"
#if !RAW_REPLAY_OLD
#include "ReceiverProfileApply.h"
#endif
#undef private
#undef protected

#if RAW_REPLAY_MOSAIC
#include "SbfParser.h"
#include "TR_GeoMag.h"
#include "TR_Orientation.h"
#include "flight_votes.h"
#include "gnss_only_arm.h"
#include "mosaic_feed.h"
#include "rinex_nav.h"
#include "sim_epoch.h"
#include "truth_nav.h"
#endif

using namespace gnss_raw;

// the carrier's counts, where the code has them (not the old rig code's)
#if RAW_REPLAY_OLD
#define CR(x) 0u
#else
#define CR(x) (x)
#endif

// --digest names fields the flown design has (the vertical check and its latch): bitident.sh's builds
#define RAW_REPLAY_DIGEST (!RAW_REPLAY_OLD && RAW_REPLAY_VCHECK && RAW_REPLAY_LATCH)

namespace
{

constexpr double kD2R = M_PI / 180.0;
constexpr double kR2D = 180.0 / M_PI;

struct Args
{
    std::string in, out;
    int heading_mode = 1;
    double pad_sigma_deg = 30.0;
    double handoff_deg = 10.0;
    int gate_full = 0;
    double tilt_sigma_deg = 0.0;
    int carrier = -1;            // -1: the profile's (the LC86G's: observe)
    int half_cycle_repair = -1;  // -1: the profile's
    double mag_error_deg = 0.0;
    double recorded_mag_error_deg = 0.0;
    double epoch_delay_ms = 100.0;  // an epoch the log has no on-board age for: handed over this long after it arrived
    bool match_ages = true;         // hand each epoch over when the on-board filter closed it (its logged age)
    double age_lead_ms = 20.0;      //   less this: the FC fused it a slice a step before that close
    // a gate switched in flight (a harness-only experiment, no firmware change): act's full gate from this flight
    // time on (the heading-only gate before it); "apogee" = the log's own apogee call
    bool gate_full_from_apogee = false;
    double gate_full_at_s = -1.0e9;
    double burnout_s = 2.755;       // the apogee votes are looked for from here (flight time, the truth's burnout)
    // act opened later than launch (a harness-only experiment): run in observe (--heading-mode 1) and at the log's
    // apogee call (or this flight time) do what act's launch edge does -- the heading's prior, the tilt floor and
    // the gate (--gate-full) -- with the gates back to the config's on the ground
    bool act_from_apogee = false;
    double act_at_s = -1.0e9;
    double epoch_shift_ms = 0.0;    // every epoch's MCU time moved by this (a receiver output delay, T1, tried)
    double diag_from_s = 1.0e9, diag_to_s = -1.0e9;   // epochs in this flight-time window: each unit printed
    double truth_t0_tow = 573786.0; // the GPS time of week of the flight's T+0 (the scenario's), for the diagnosis
    // boost safeguards (harness-only experiments), for an epoch whose IMU peak since the last one is above
    // guard_g (|f| > (1 + guard_g) g): the attitude held out of it with fewer than min_pr pseudoranges (a1) or when
    // its Dopplers do not agree about one velocity to dop_rms_mps (a2); a satellite refused that would turn the
    // heading past step_deg in the epoch (b2) or past step_sigma of its 1-sigma (b1); the innovation gate tightened
    // to gate_chi2 above tight_g (c)
    double guard_g = 1.0;
    int min_pr = 0;
    double dop_rms_mps = 0.0;
    double dv_max_mps = 0.0;        // (a3) or the velocity correction the epoch's Dopplers point to is above this
    double dv_sigma = 0.0;          // ... or above this many of the filter's own velocity 1-sigma, if that is larger
    bool reject_epoch = false;      // (a4) such an epoch dropped whole, not only held from the attitude
    bool majority = false;          // (a6) a boost epoch most of whose range rates the gate refuses: dropped whole
    bool majority_half = false;     // (a6h) ... half or more of them (a tie drops it too)
    bool dop_thin_pass = false;     // (a2/a3) an epoch with too few Dopplers to judge passes (default: held out)
    double dv_vert_max = 0.0;       // (a3v) ... or whose fitted velocity is off the filter's VERTICAL by more than this
                                    //   (m/s): the vertical is the IMU's and barometer's, whatever the heading
    double dv_vert_hold = 0.0;      // (a3vh) ... above this (and not past dv_vert_max): the attitude held out of the
                                    //   epoch, its velocity and position fused (never dropped, whatever --guard-reject-epoch)
    double step_deg = 0.0, step_sigma = 0.0;
    double tight_g = 2.0, gate_chi2 = 0.0;
    // a boost dropout, synthesized: every epoch whose GPS time falls in [cut_from_s, cut_to_s) of flight time is
    // dropped; and one epoch (the one nearest corrupt_at_s) corrupted like c30's T+1: on 3 of its satellites the
    // range rate off by 30..70 m/s and the pseudorange by 20..40 m, signs and satellites drawn from corrupt_seed.
    // Flight time is the epoch's GPS time of week less truth_t0_tow (the rig scenario's), or, on a mosaic or cold
    // start replay without --truth-t0-tow, its MCU time plus t_base_s (the CSV's t_s)
    double cut_from_s = 1.0e9, cut_to_s = 1.0e9;
    // the relock rule: before the flight computer's apogee call, a satellite whose lock time (MSM7's) is under
    // this many ms gives no range rate -- a relocked loop's Doppler settles over a second or two
    double relock_min_lock_ms = 0.0;
    double corrupt_at_s = -1.0e9;
    int corrupt_seed = 1;
    // the corruption's size, drawn uniform in [lo, hi] with a random sign: the range rate (m/s) and the
    // pseudorange (m); the defaults are c30's gross errors, which the gate refuses
    double corrupt_rr_mps[2] = {30.0, 70.0};
    double corrupt_pr_m[2] = {20.0, 40.0};
    // the corrupted epoch is the first at or after corrupt_at_s on a whole GPS second (its pseudoranges fused there,
    // pr_period_s 1 s), not the nearest
    bool corrupt_pr_epoch = false;
    double fc_unit_us = 0.0;     // >0: an emulated FC clock -- each job unit costs this, so epochs slice as on the FC
    bool reanchor = false;       // at the launch tick, take the pad context's state and covariance as they were
    double no_line_ms = 25.0;    // a log with no clock line at all (no fix in it): an epoch's MCU time is its last
                                 // frame's arrival plus this (the line sat 10-47 ms, median 25, after it on r20-r25);
                                 // negative: leave such epochs untimed
    double pad_lat = 39.4683565, pad_lon = -75.29286495, pad_h = -0.712;   // the truth's pad (NED origin)
    double t_base_s = 0.0;       // flight time = MCU s + this (the ignition edge)
    // a calibrated IMU (the pad's hand calibration): these subtracted from every tick, and the pad state the
    // filter would have had with it (its attitude, accelerometer bias, the gyro bias it had less what came off)
    double acc_sub[3] = {}, gyro_sub[3] = {};
    bool have_pad_quat = false, have_pad_ab = false;
    double pad_quat[4] = {1, 0, 0, 0}, pad_ab[3] = {}, pad_wb_sub[3] = {};
    int every = 1;               // a CSV row every this many steps
    // the firmware's own switches since fc/gnss-heading-fuse 0729114f (-1: the firmware's default)
    int boost_screen = -1;           // RawGnssShadow::Config::boost_screen: 0 off, 1 in act (default), 2 always
    double boost_screen_g = -1.0;    //   boost_screen_g (2 g)
    double pad_sigma_fallback_deg = -1.0;   // heading_pad_sigma_fallback_deg (30): act's prior without the
                                            //   magnetometer's heading (--pad-sigma-deg: with it)
    double mag_recent_s = -1.0;      // heading_mag_recent_s (5 s)
    int level_heading_free = -1;     // RawGnssEKFParams::level_heading_free (1)
    int mag_flags = 2;               // each magnetometer sample's ShadowMag flags: 2 the rig's field (a sim's, the
                                     //   truth's: kShadowMagSim), 1 calibrated, 0 neither (refused)
    // compare the firmware's boost screen (a pre-pass at the epoch's prior) with the harness's sequential majority
    // check (the epoch worked, then its refusals counted) on every boost epoch, each from the same state
    bool compare_sequential = false;
    bool apogee_flag = true;         // kShadowApogee on every tick from the log's apogee flag (--no-apogee-flag: never)
    double vert_check_mps = -1.0;    // RawGnssShadow::Config::vert_check_mps (20 m/s; 0 off) -- a RAW_REPLAY_VCHECK build
    double boost_screen_window_s = -1.0;   // boost_screen_window_s (1.5 s) -- a RAW_REPLAY_VCHECK build
    double rr_dyn_sigma = -1.0;      // the receiver profile's rr_dyn_sigma (the Doppler's allowance per g)
    double rr_fit_gate = -1.0;       // RawGnssEKFParams::rr_fit_gate (the Dopplers judged by their epoch's fit) -- a
                                     //   RAW_REPLAY_FITGATE build
    double vert_release_mps = -1.0;  // RawGnssShadow::Config::vert_release_mps (the vertical check's latch) -- a
                                     //   RAW_REPLAY_LATCH build
    double rr_dyn_tau_s = -1.0;      // the receiver profile's rr_dyn_tau_s (the Doppler's settling) -- a
                                     //   RAW_REPLAY_DYNMEM build
    double max_epoch_age_s = -1.0;   // RawGnssShadow::Config::max_epoch_age_s (0.6 s): later, an epoch is dropped
    std::string digest;              // a line of hashes per filter step (bitident.sh)
    bool fw_defaults = false;        // the heading's settings at the firmware's defaults unless given (bitident.sh)
    // the V8/V9 with the mosaic-G5 (RAW_REPLAY_MOSAIC builds)
    std::string receiver = "lc86g";  // --receiver (--profile): the receiver profile and the build it flies with
    bool cold_start = false;         // no S/PNAV/PCOV/A: the filter starts on the pad from the receiver's fix
    bool unaided = false;            // RawGnssEKFParams::att_gnss_off: the twin the acceptance compares against
    std::vector<std::string> rinex;  // RINEX 3 navigation files: the day's broadcast ephemerides
    bool quorum = false;             // the apogee quorum (always with the mosaic)
    int vote_guard = -1;             // the raw vote guard withholds the filter's votes (-1: the firmware's: no, log only)
    double loop_hz = 1000.0;         // the flight loop's passes a second (config::FLIGHT_LOOP_UPDATE_RATE)
    double seed_heading_deg = 0.0;   // the cold start's heading guess (config::PAD_HEADING_DEG)
    double seed_heading_error_deg = 0.0;   // ... and a sim's: its seed (META seed_quat) turned this far off about down
    std::string truth_nav;           // a sim's truth (sim_events.py truth/<flight>.nav): a second quorum on it
    // RawGnssEKFParams::hdg_obs_* and RawGnssShadow::Config::heading_obs_scope (a RAW_REPLAY_HDGOBS build): GNSS's
    // heading gain by how well the moment observes it (-1 / below 0: the firmware's, off)
    int hdg_obs_mode = -1;
    double hdg_obs_fh = -1.0, hdg_obs_tau = -1.0, hdg_obs_sigma_v = -1.0, hdg_obs_window = -1.0;
    double hdg_obs_open_g = -1.0, hdg_obs_open_hold = -1.0, hdg_obs_floor = -1.0;
    int hdg_obs_scope = -1;
    int reject_persist_rr = -1;      // RawGnssEKFParams::reject_persist_rr after the profile (-1: the profile's; the
                                     //   mosaic's 30 epochs, 3 s at 10 Hz): a Doppler refused this many epochs running
                                     //   is let back in
    bool have_seed_gyro = false;     // --seed-gyro-dps: the cold start's gyro bias given, not the ring's mean
    double seed_gyro_dps[3] = {};    //   (body FRD, deg/s): a log that begins while the payload turns
    int sat_cap = -1;                // TR_BENCH_RAW_SAT_CAP: the stage's satellites an epoch (-1: the profile's)
    int cap_max = -1;                // TR_BENCH_RAW_CAP_MAX: the cap governor's ceiling (-1: the profile's, off)
    double slice_us = -1.0;          // TR_BENCH_RAW_SLICE_US: the filter's slice a step (-1: the profile's 2500)
    double fuse_ms = -1.0;           // TR_BENCH_RAW_FUSE_MS: the fused epochs' period (-1: the profile's)
    int share_los = -1;              // TR_BENCH_RAW_SHARE_LOS: 1 the epoch's geometry shared by its units, 2 and the
                                     //   Doppler's prediction in float (-1: the profile's, off; a RAW_REPLAY_SHARELOS
                                     //   build)
    // the Dopplers in pairs (TR_BENCH_RAW_DOP_AVG, the profile's rr_pair_avg; a RAW_REPLAY_DOPAVG build): each grid
    // epoch's averaged with the receiver's epoch before it, as the driver does (MosaicRawEpoch.h), the filter taking
    // the mean half an epoch earlier; -1: the profile's (off)
    int rr_pair_avg = -1;
    double rr_pair_max_diff = -1.0;  //   the pair at most this far apart, m/s (-1: the profile's 15)
    double rr_pair_sigma = -1.0;     //   offline only: each averaged Doppler's sigma scaled by this (no firmware path)
    bool sim_fuse_grid = false;      // a sim's epochs (X) cut to the fused grid as the driver cuts the receiver's
    double rr_lag_ms = -1.0;         // offline only: the filter's rr_lag_s in place of the profile's, ms (-1: the profile's)
    bool sim_pair_exact = false;     // offline only, a sim with --truth-nav: each averaged Doppler made the range rate
                                     //   at the pair's middle exactly (the truth's second-order term taken out)
                                     //   (implied by --rr-pair-avg 1: the epochs between are read, not fused)
    std::string flight_state;        // the launch and landing: "log" (the ticks' flags), "quorum" (the quorum's own
                                     //   state machine); default the ticks', the quorum's where META state_machine none
    bool time_tag_from_boot = false; // the mosaic's time tag as after a boot: untimed until the log's first
                                     //   ReceiverTime says FINETIME and its first K line (a 1PPS pairing)
    bool assume_finetime = false;    // a log with no ReceiverTime at all: its epochs timed as FINETIME
    int pad_zupt = -1;               // RawGnssShadow::Config::pad_zupt (-1: the firmware's, on)
    double gnss_only_from_s = 1.0e9; // a static log scored on GNSS: from this flight time on the ground, no pad ZUPT
                                     //   and no levelling (the IMU, GNSS and the barometer alone, as in flight)
    int gnss_only_gate = 1;          //   and act's launch edge on the attitude (1, launch: gnss_only_arm.h), or the
                                     //   ground's gate kept (0, ground)
    bool gnss_only_keep_level = false; //   with the levelling kept on: the descent as flown (gnss_only_arm.h)
    std::string pvt_out;             // each receiver fix's height above the ellipsoid, for the pad height's score
    // the Doppler change check (RawGnssShadow::Config::dv_*; a RAW_REPLAY_DVCHECK build; -1: the firmware's)
    int dv_check = -1;               // 0 off (the firmware's), 1 observe, 2 act
    double dv_sat_k = -1.0, dv_epoch_k = -1.0, dv_floor = -1.0, dv_per_g = -1.0, dv_att_deg = -1.0,
           dv_scale = -1.0, dv_bias = -1.0, dv_lever_m = -1.0, dv_hold_s = -1.0, dv_epoch_hold_s = -1.0,
           dv_window_s = -1.0, dv_noise_tau_s = -1.0, dv_hold_all_frac = -1.0, dv_latch_k = -1.0;
    double dv_hdg_unset_deg = -1.0;  // a launch with no magnetometer heading: the heading allowance (a RAW_REPLAY_DVHDG build)
    int dv_deweight = -1, dv_ground = -1;
    // the turning body (RawGnssShadow::Config, a RAW_REPLAY_ROT build; -1: the firmware's, off): the antenna's
    // offset from the IMU bounded along the nose and across it (each Doppler's sigma takes rotLeverSigma), and
    // GNSS held off the attitude from a rate across the nose and for a while after
    double rot_lever_axial_m = -1.0, rot_lever_radial_m = -1.0, rot_lever_min_rps = -1.0;
    double att_hold_rate_rps = -1.0, att_hold_s = -1.0;
    double rot_skip_rate_rps = -1.0, rot_skip_s = -1.0;   // or nothing fused from a rate across the nose
    bool rot_after_apogee = false;   // all three only once the flight computer has called apogee
    // the antenna's offset, known (RawGnssShadow.h, applyAntennaOffset; a RAW_REPLAY_ANTENNA build): body FRD, m,
    // and what it could be off by (-1: the firmware's, the mosaic safeguards'), from a rate across the nose, and the
    // lever's attitude terms (LeverAttMode: 0 off, 1 observe, 2 fuse)
    bool ant_known = false;
    bool ant_none = false;           // --antenna-m none: unknown, whatever the events file's META antenna_m says
    double ant_m[3] = {0.0, 0.0, 0.0};
    double ant_sigma_axial_m = -1.0, ant_sigma_radial_m = -1.0, ant_min_rps = -1.0;
    int ant_att_mode = -1;
    // the spinning body (a RAW_REPLAY_WINDUP build; -1: the firmware's, off): the carrier wind-up taken off each
    // Doppler from the gyro (RawGnssShadow::Config::windup_k, cycles a turn), Wu's whole rate, the monitor; the
    // covariance's exact attitude transition (RawGnssEKFParams::att_exact); the change check's lever mode
    double windup_k = -1.0;
    int windup_geom = -1, windup_monitor = -1, att_exact = -1, dv_lever_mode = -1;
    double windup_monitor_k = -1.0, dv_lever_radial_m = -1.0;
    double clk_drift_psd = -1.0;     // RawGnssEKFParams::clk_drift_psd after the profile (-1: the profile's; the
                                     //   mosaic's and the LC86G's 100 m^2/s^3): an experiment on the clock's walk
    bool build_lite = false;         // TR_BENCH_RAW_BUILD_LITE: the cheaper epoch build (a RAW_REPLAY_BUILD_LITE build)
    std::set<std::string> given;     // the options on the command line
};

void usage()
{
    std::fprintf(stderr,
                 "raw_replay --in EVENTS.txt --out RUN.csv [--heading-mode 0|1|2] [--pad-sigma-deg X]\n"
                 "  [--handoff-deg X] [--gate-full 0|1] [--tilt-sigma-deg X] [--carrier 0|1|2]\n"
                 "  [--half-cycle-repair 0|1] [--mag-error-deg X] [--recorded-mag-error-deg X]\n"
                 "  [--epoch-delay-ms X] [--no-match-ages] [--age-lead-ms X] [--fc-unit-us X] [--reanchor]\n"
                 "  [--t-base-s X] [--pad lat,lon,h] [--every N]\n"
                 "  [--acc-sub x,y,z] [--gyro-sub x,y,z] [--pad-quat w,x,y,z] [--pad-ab x,y,z] [--pad-wb-sub x,y,z]\n"
                 "  [--gate-full-from-apogee] [--gate-full-at-s X] [--burnout-s X] [--act-from-apogee] [--act-at-s X]\n"
                 "  [--epoch-shift-ms X] [--diag-from-s X --diag-to-s Y]\n"
                 "  [--guard-g X] [--guard-min-pr N] [--guard-doppler-rms X] [--guard-dv-max X] [--guard-dv-sigma K]\n"
                 "  [--guard-reject-epoch] [--guard-majority] [--guard-majority-half] [--guard-step-deg X]\n"
                 "  [--guard-step-sigma X] [--relock-min-lock-ms X] [--guard-dop-thin-pass] [--guard-dv-vert-max X]\n"
                 "  [--guard-dv-vert-hold X]\n"
                 "  [--guard-tight-g X --guard-gate-chi2 X] [--cut-from-s X --cut-to-s Y] [--corrupt-at-s X --corrupt-seed N]\n"
                 "  [--corrupt-rr-mps lo,hi] [--corrupt-pr-m lo,hi] [--corrupt-pr-epoch]\n"
                 "  [--boost-screen 0|1|2] [--boost-screen-g X] [--pad-sigma-fallback-deg X] [--mag-recent-s X]\n"
                 "  [--level-heading-free 0|1] [--mag-flags N] [--compare-sequential] [--no-apogee-flag]\n"
                 "  [--max-epoch-age-s X] [--vert-check-mps X] [--boost-screen-window-s X] [--rr-dyn-tau-s X]\n"
                 "  [--rr-dyn-sigma X] [--vert-release-mps X] [--rr-fit-gate X] [--fw-defaults] [--digest FILE]\n"
                 "  [--truth-t0-tow X]\n"
                 "  the mosaic-G5 (a RAW_REPLAY_MOSAIC build): [--receiver lc86g|mosaic] [--cold-start] [--unaided]\n"
                 "  [--rinex FILE]... [--quorum] [--vote-guard 0|1] [--loop-hz X] [--seed-heading-deg X]\n"
                 "  [--seed-heading-error-deg X] [--reject-persist-rr N] [--truth-nav FILE]\n"
                 "  [--seed-gyro-dps x,y,z]\n"
                 "  [--sat-cap N] [--fuse-ms X] [--flight-state log|quorum] [--time-tag-from-boot]\n"
                 "  [--rr-pair-avg 0|1] [--rr-pair-max-diff X] [--rr-pair-sigma X] [--sim-fuse-grid] [--rr-lag-ms X]\n"
                 "  [--sim-pair-exact]\n"
                 "  [--assume-finetime] [--pad-zupt 0|1] [--gnss-only-from-s X] [--gnss-only-gate launch|ground]\n"
                 "  [--pvt-out FILE] [--gnss-only-keep-level]\n"
                 "  the epoch's geometry (a RAW_REPLAY_SHARELOS build): [--share-los 0|1|2]\n"
                 "  the satellite cap's governor and the profile's slice (a RAW_REPLAY_CAPGOV build): [--cap-max N]\n"
                 "  [--slice-us X]\n"
                 "  the Doppler change check (a RAW_REPLAY_DVCHECK build): [--dv-check 0|1|2] [--dv-sat-k X]\n"
                 "  [--dv-epoch-k X] [--dv-floor X] [--dv-per-g X] [--dv-att-deg X] [--dv-scale X] [--dv-bias X]\n"
                 "  [--dv-lever-m X] [--dv-hold-s X] [--dv-epoch-hold-s X] [--dv-window-s X] [--dv-deweight 0|1]\n"
                 "  [--dv-ground 0|1] [--dv-noise-tau-s X] [--dv-hold-all-frac X] [--dv-latch-k X]\n"
                 "  [--dv-hdg-unset-deg X]\n"
                 "  the heading's gain by its observability (a RAW_REPLAY_HDGOBS build): [--hdg-obs-mode 0|1|2|3]\n"
                 "  [--hdg-obs-fh X] [--hdg-obs-tau X] [--hdg-obs-sigma-v X] [--hdg-obs-window X] [--hdg-obs-open-g X]\n"
                 "  [--hdg-obs-open-hold X] [--hdg-obs-floor X] [--hdg-obs-scope 0|1|2]\n"
                 "  the turning body (a RAW_REPLAY_ROT build): [--rot-lever-axial-m X] [--rot-lever-radial-m X]\n"
                 "  [--rot-lever-min-rate X]\n"
                 "  [--att-hold-rate X] [--att-hold-s X] [--rot-skip-rate X] [--rot-skip-s X]\n"
                 "  [--rot-after-apogee]\n"
                 "  the antenna's offset (a RAW_REPLAY_ANTENNA build): [--antenna-m x,y,z|none] (default: the events\n"
                 "  file's META antenna_m, what the flight flew with, else unknown) [--antenna-sigma-m axial,radial]\n"
                 "  [--antenna-min-rate X] [--antenna-att-mode 0|1|2]\n"
                 "  the spinning body (a RAW_REPLAY_WINDUP build): [--windup-k K] [--windup-geom 0|1]\n"
                 "  [--windup-monitor 0|1] [--windup-monitor-k K] [--att-exact 0|1] [--dv-lever-mode 0|1]\n"
                 "  [--dv-lever-radial-m X] [--clk-drift-psd X]\n"
                 "  the cheaper epoch build (a RAW_REPLAY_BUILD_LITE build, --receiver mosaic only): [--build-lite]\n");
}

bool parseArgs(int argc, char** argv, Args& a)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string k = argv[i];
        a.given.insert(k);
        auto val = [&](void) -> const char* {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "%s needs a value\n", k.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (k == "--in") a.in = val();
        else if (k == "--out") a.out = val();
        else if (k == "--heading-mode") a.heading_mode = std::atoi(val());
        else if (k == "--pad-sigma-deg") a.pad_sigma_deg = std::atof(val());
        else if (k == "--handoff-deg") a.handoff_deg = std::atof(val());
        else if (k == "--gate-full") a.gate_full = std::atoi(val());
        else if (k == "--tilt-sigma-deg") a.tilt_sigma_deg = std::atof(val());
        else if (k == "--carrier") a.carrier = std::atoi(val());
        else if (k == "--half-cycle-repair") a.half_cycle_repair = std::atoi(val());
        else if (k == "--mag-error-deg") a.mag_error_deg = std::atof(val());
        else if (k == "--recorded-mag-error-deg") a.recorded_mag_error_deg = std::atof(val());
        else if (k == "--epoch-delay-ms") a.epoch_delay_ms = std::atof(val());
        else if (k == "--no-match-ages") a.match_ages = false;
        else if (k == "--gate-full-from-apogee") a.gate_full_from_apogee = true;
        else if (k == "--act-from-apogee") a.act_from_apogee = true;
        else if (k == "--epoch-shift-ms") a.epoch_shift_ms = std::atof(val());
        else if (k == "--diag-from-s") a.diag_from_s = std::atof(val());
        else if (k == "--guard-g") a.guard_g = std::atof(val());
        else if (k == "--guard-min-pr") a.min_pr = std::atoi(val());
        else if (k == "--guard-doppler-rms") a.dop_rms_mps = std::atof(val());
        else if (k == "--guard-dv-max") a.dv_max_mps = std::atof(val());
        else if (k == "--guard-dv-sigma") a.dv_sigma = std::atof(val());
        else if (k == "--guard-reject-epoch") a.reject_epoch = true;
        else if (k == "--guard-dop-thin-pass") a.dop_thin_pass = true;
        else if (k == "--guard-dv-vert-max") a.dv_vert_max = std::atof(val());
        else if (k == "--guard-dv-vert-hold") a.dv_vert_hold = std::atof(val());
        else if (k == "--guard-majority") a.majority = true;
        else if (k == "--guard-majority-half") a.majority = a.majority_half = true;
        else if (k == "--cut-from-s") a.cut_from_s = std::atof(val());
        else if (k == "--relock-min-lock-ms") a.relock_min_lock_ms = std::atof(val());
        else if (k == "--cut-to-s") a.cut_to_s = std::atof(val());
        else if (k == "--corrupt-at-s") a.corrupt_at_s = std::atof(val());
        else if (k == "--corrupt-seed") a.corrupt_seed = std::atoi(val());
        else if (k == "--corrupt-rr-mps")
        {
            if (std::sscanf(val(), "%lf,%lf", &a.corrupt_rr_mps[0], &a.corrupt_rr_mps[1]) != 2)
                return false;
        }
        else if (k == "--corrupt-pr-m")
        {
            if (std::sscanf(val(), "%lf,%lf", &a.corrupt_pr_m[0], &a.corrupt_pr_m[1]) != 2)
                return false;
        }
        else if (k == "--corrupt-pr-epoch") a.corrupt_pr_epoch = true;
        else if (k == "--guard-step-deg") a.step_deg = std::atof(val());
        else if (k == "--guard-step-sigma") a.step_sigma = std::atof(val());
        else if (k == "--guard-tight-g") a.tight_g = std::atof(val());
        else if (k == "--guard-gate-chi2") a.gate_chi2 = std::atof(val());
        else if (k == "--diag-to-s") a.diag_to_s = std::atof(val());
        else if (k == "--act-at-s") a.act_at_s = std::atof(val());
        else if (k == "--gate-full-at-s") a.gate_full_at_s = std::atof(val());
        else if (k == "--burnout-s") a.burnout_s = std::atof(val());
        else if (k == "--age-lead-ms") a.age_lead_ms = std::atof(val());
        else if (k == "--fc-unit-us") a.fc_unit_us = std::atof(val());
        else if (k == "--reanchor") a.reanchor = true;
        else if (k == "--no-line-ms") a.no_line_ms = std::atof(val());
        else if (k == "--t-base-s") a.t_base_s = std::atof(val());
        else if (k == "--every") a.every = std::max(1, std::atoi(val()));
        else if (k == "--boost-screen") a.boost_screen = std::atoi(val());
        else if (k == "--boost-screen-g") a.boost_screen_g = std::atof(val());
        else if (k == "--pad-sigma-fallback-deg") a.pad_sigma_fallback_deg = std::atof(val());
        else if (k == "--mag-recent-s") a.mag_recent_s = std::atof(val());
        else if (k == "--level-heading-free") a.level_heading_free = std::atoi(val());
        else if (k == "--mag-flags") a.mag_flags = std::atoi(val());
        else if (k == "--compare-sequential") a.compare_sequential = true;
        else if (k == "--no-apogee-flag") a.apogee_flag = false;
        else if (k == "--max-epoch-age-s") a.max_epoch_age_s = std::atof(val());
        else if (k == "--vert-check-mps") a.vert_check_mps = std::atof(val());
        else if (k == "--boost-screen-window-s") a.boost_screen_window_s = std::atof(val());
        else if (k == "--rr-dyn-tau-s") a.rr_dyn_tau_s = std::atof(val());
        else if (k == "--rr-dyn-sigma") a.rr_dyn_sigma = std::atof(val());
        else if (k == "--vert-release-mps") a.vert_release_mps = std::atof(val());
        else if (k == "--rr-fit-gate") a.rr_fit_gate = std::atof(val());
        else if (k == "--digest") a.digest = val();
        else if (k == "--fw-defaults") a.fw_defaults = true;
        else if (k == "--truth-t0-tow") a.truth_t0_tow = std::atof(val());
        else if (k == "--receiver" || k == "--profile") a.receiver = val();
        else if (k == "--cold-start") a.cold_start = true;
        else if (k == "--unaided") a.unaided = true;
        else if (k == "--rinex") a.rinex.push_back(val());
        else if (k == "--quorum") a.quorum = true;
        else if (k == "--vote-guard") a.vote_guard = std::atoi(val());
        else if (k == "--loop-hz") a.loop_hz = std::atof(val());
        else if (k == "--seed-heading-deg") a.seed_heading_deg = std::atof(val());
        else if (k == "--seed-heading-error-deg") a.seed_heading_error_deg = std::atof(val());
        else if (k == "--reject-persist-rr") a.reject_persist_rr = std::atoi(val());
        else if (k == "--gnss-only-keep-level") a.gnss_only_keep_level = true;
        else if (k == "--dv-check") a.dv_check = std::atoi(val());
        else if (k == "--dv-sat-k") a.dv_sat_k = std::atof(val());
        else if (k == "--dv-epoch-k") a.dv_epoch_k = std::atof(val());
        else if (k == "--dv-floor") a.dv_floor = std::atof(val());
        else if (k == "--dv-per-g") a.dv_per_g = std::atof(val());
        else if (k == "--dv-att-deg") a.dv_att_deg = std::atof(val());
        else if (k == "--dv-scale") a.dv_scale = std::atof(val());
        else if (k == "--dv-bias") a.dv_bias = std::atof(val());
        else if (k == "--dv-lever-m") a.dv_lever_m = std::atof(val());
        else if (k == "--dv-hold-s") a.dv_hold_s = std::atof(val());
        else if (k == "--dv-epoch-hold-s") a.dv_epoch_hold_s = std::atof(val());
        else if (k == "--dv-window-s") a.dv_window_s = std::atof(val());
        else if (k == "--dv-deweight") a.dv_deweight = std::atoi(val());
        else if (k == "--dv-ground") a.dv_ground = std::atoi(val());
        else if (k == "--dv-noise-tau-s") a.dv_noise_tau_s = std::atof(val());
        else if (k == "--dv-hold-all-frac") a.dv_hold_all_frac = std::atof(val());
        else if (k == "--dv-latch-k") a.dv_latch_k = std::atof(val());
        else if (k == "--dv-hdg-unset-deg") a.dv_hdg_unset_deg = std::atof(val());
        else if (k == "--hdg-obs-mode") a.hdg_obs_mode = std::atoi(val());
        else if (k == "--hdg-obs-fh") a.hdg_obs_fh = std::atof(val());
        else if (k == "--hdg-obs-tau") a.hdg_obs_tau = std::atof(val());
        else if (k == "--hdg-obs-sigma-v") a.hdg_obs_sigma_v = std::atof(val());
        else if (k == "--hdg-obs-window") a.hdg_obs_window = std::atof(val());
        else if (k == "--hdg-obs-open-g") a.hdg_obs_open_g = std::atof(val());
        else if (k == "--hdg-obs-open-hold") a.hdg_obs_open_hold = std::atof(val());
        else if (k == "--hdg-obs-floor") a.hdg_obs_floor = std::atof(val());
        else if (k == "--hdg-obs-scope") a.hdg_obs_scope = std::atoi(val());
        else if (k == "--truth-nav") a.truth_nav = val();
        else if (k == "--seed-gyro-dps")
        {
            a.have_seed_gyro = std::sscanf(val(), "%lf,%lf,%lf", &a.seed_gyro_dps[0], &a.seed_gyro_dps[1],
                                           &a.seed_gyro_dps[2]) == 3;
        }
        else if (k == "--sat-cap") a.sat_cap = std::atoi(val());
        else if (k == "--build-lite") a.build_lite = true;
        else if (k == "--cap-max") a.cap_max = std::atoi(val());
        else if (k == "--slice-us") a.slice_us = std::atof(val());
        else if (k == "--fuse-ms") a.fuse_ms = std::atof(val());
        else if (k == "--share-los") a.share_los = std::atoi(val());
        else if (k == "--rr-pair-avg") a.rr_pair_avg = std::atoi(val());
        else if (k == "--rr-pair-max-diff") a.rr_pair_max_diff = std::atof(val());
        else if (k == "--rr-pair-sigma") a.rr_pair_sigma = std::atof(val());
        else if (k == "--sim-fuse-grid") a.sim_fuse_grid = true;
        else if (k == "--rr-lag-ms") a.rr_lag_ms = std::atof(val());
        else if (k == "--sim-pair-exact") a.sim_pair_exact = true;
        else if (k == "--flight-state") a.flight_state = val();
        else if (k == "--time-tag-from-boot") a.time_tag_from_boot = true;
        else if (k == "--assume-finetime") a.assume_finetime = true;
        else if (k == "--pad-zupt") a.pad_zupt = std::atoi(val());
        else if (k == "--gnss-only-from-s") a.gnss_only_from_s = std::atof(val());
        else if (k == "--gnss-only-gate")
        {
            const std::string g = val();
            if (g == "launch")
                a.gnss_only_gate = 1;
            else if (g == "ground")
                a.gnss_only_gate = 0;
            else
                return false;
        }
        else if (k == "--pvt-out") a.pvt_out = val();
        else if (k == "--rot-lever-axial-m") a.rot_lever_axial_m = std::atof(val());
        else if (k == "--rot-lever-radial-m") a.rot_lever_radial_m = std::atof(val());
        else if (k == "--rot-lever-min-rate") a.rot_lever_min_rps = std::atof(val());
        else if (k == "--att-hold-rate") a.att_hold_rate_rps = std::atof(val());
        else if (k == "--att-hold-s") a.att_hold_s = std::atof(val());
        else if (k == "--rot-skip-rate") a.rot_skip_rate_rps = std::atof(val());
        else if (k == "--rot-skip-s") a.rot_skip_s = std::atof(val());
        else if (k == "--rot-after-apogee") a.rot_after_apogee = true;
        else if (k == "--antenna-m")
        {
            const char* v = val();
            if (std::strcmp(v, "none") == 0)
                a.ant_none = true;
            else if (std::sscanf(v, "%lf,%lf,%lf", &a.ant_m[0], &a.ant_m[1], &a.ant_m[2]) != 3)
                return false;
            else
                a.ant_known = true;
        }
        else if (k == "--antenna-sigma-m")
        {
            if (std::sscanf(val(), "%lf,%lf", &a.ant_sigma_axial_m, &a.ant_sigma_radial_m) != 2)
                return false;
        }
        else if (k == "--antenna-min-rate") a.ant_min_rps = std::atof(val());
        else if (k == "--antenna-att-mode") a.ant_att_mode = std::atoi(val());
        else if (k == "--windup-k") a.windup_k = std::atof(val());
        else if (k == "--windup-geom") a.windup_geom = std::atoi(val());
        else if (k == "--windup-monitor") a.windup_monitor = std::atoi(val());
        else if (k == "--windup-monitor-k") a.windup_monitor_k = std::atof(val());
        else if (k == "--att-exact") a.att_exact = std::atoi(val());
        else if (k == "--dv-lever-mode") a.dv_lever_mode = std::atoi(val());
        else if (k == "--dv-lever-radial-m") a.dv_lever_radial_m = std::atof(val());
        else if (k == "--clk-drift-psd") a.clk_drift_psd = std::atof(val());
        else if (k == "--acc-sub")
        {
            if (std::sscanf(val(), "%lf,%lf,%lf", &a.acc_sub[0], &a.acc_sub[1], &a.acc_sub[2]) != 3)
                return false;
        }
        else if (k == "--gyro-sub")
        {
            if (std::sscanf(val(), "%lf,%lf,%lf", &a.gyro_sub[0], &a.gyro_sub[1], &a.gyro_sub[2]) != 3)
                return false;
        }
        else if (k == "--pad-quat")
        {
            if (std::sscanf(val(), "%lf,%lf,%lf,%lf", &a.pad_quat[0], &a.pad_quat[1], &a.pad_quat[2],
                            &a.pad_quat[3]) != 4)
                return false;
            a.have_pad_quat = true;
        }
        else if (k == "--pad-ab")
        {
            if (std::sscanf(val(), "%lf,%lf,%lf", &a.pad_ab[0], &a.pad_ab[1], &a.pad_ab[2]) != 3)
                return false;
            a.have_pad_ab = true;
        }
        else if (k == "--pad-wb-sub")
        {
            if (std::sscanf(val(), "%lf,%lf,%lf", &a.pad_wb_sub[0], &a.pad_wb_sub[1], &a.pad_wb_sub[2]) != 3)
                return false;
        }
        else if (k == "--pad")
        {
            if (std::sscanf(val(), "%lf,%lf,%lf", &a.pad_lat, &a.pad_lon, &a.pad_h) != 3)
                return false;
        }
        else
        {
            std::fprintf(stderr, "unknown argument %s\n", k.c_str());
            return false;
        }
    }
    return !a.in.empty() && !a.out.empty();
}

std::vector<uint8_t> unhex(const std::string& s)
{
    std::vector<uint8_t> out(s.size() / 2);
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = static_cast<uint8_t>(std::stoi(s.substr(2 * i, 2), nullptr, 16));
    return out;
}

// One event from extract.py, in time order.
struct Event
{
    char kind;
    int64_t t_us;
    double v[8];
    int flags;
    std::vector<uint8_t> bytes;
};

struct Input
{
    std::map<std::string, std::string> meta;
    std::vector<double> s;          // S: the first record's state
    std::vector<double> pnav;       // PNAV: the pad context's nav
    std::vector<double> pcov;       // PCOV: 190 values, upper triangle
    std::vector<std::vector<uint8_t>> peph;
    std::map<uint32_t, double> age_ms;   // A: each epoch's age when the on-board filter closed it
    std::vector<Event> ev;
    std::vector<gps_eph_t> neph;         // N: ephemerides as text (each N event's flags index it)
#if RAW_REPLAY_MOSAIC
    std::vector<rr::SimEpoch> xep;       // X: a sim's epochs, its satellites placed (each X event's flags index it)
#endif
};

// An 'N' line's ephemeris: sys prn week iode iodc health ura, then toe toc and
// the orbit and clock in gps_eph_t's units (rad, s), as rinex_nav.cpp fills it.
bool parseEph(std::istringstream& ss, gps_eph_t& e)
{
    std::memset(&e, 0, sizeof(e));
    ss >> e.sys >> e.prn >> e.week >> e.iode >> e.iodc >> e.health >> e.ura >> e.toe >> e.toc >> e.sqrt_a >> e.e >>
        e.i0 >> e.omega0 >> e.omega >> e.m0 >> e.delta_n >> e.idot >> e.omega_dot >> e.cuc >> e.cus >> e.crc >> e.crs >>
        e.cic >> e.cis >> e.af0 >> e.af1 >> e.af2 >> e.tgd;
    if (!ss)
        return false;
    e.valid = 1;
    return true;
}

bool readInput(const std::string& path, Input& in)
{
    std::ifstream f(path);
    if (!f)
        return false;
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "META")
        {
            std::string k, v;
            ss >> k;
            std::getline(ss, v);
            in.meta[k] = v.size() && v[0] == ' ' ? v.substr(1) : v;
        }
        else if (tag == "S" || tag == "PNAV" || tag == "PCOV")
        {
            std::vector<double>& dst = tag == "S" ? in.s : (tag == "PNAV" ? in.pnav : in.pcov);
            double x;
            while (ss >> x)
                dst.push_back(x);
        }
        else if (tag == "PEPH")
        {
            std::string h;
            ss >> h;
            in.peph.push_back(unhex(h));
        }
        else if (tag == "A")
        {
            long long tow = 0;
            double age = 0.0;
            ss >> tow >> age;
            in.age_ms[static_cast<uint32_t>(tow)] = age;
        }
        else if (tag == "PSUR")
        {
            // the pad survey: used only where it took the ionosphere out (checked below)
            in.meta["psur"] = line;
        }
        else if (tag.size() == 1)
        {
            Event e{};
            e.kind = tag[0];
            long long t = 0;
            ss >> t;
            e.t_us = t;
            if (e.kind == 'I')
            {
                for (int i = 0; i < 6; ++i)
                    ss >> e.v[i];
                ss >> e.flags;
            }
            else if (e.kind == 'B')
                ss >> e.v[0];
            else if (e.kind == 'M')
                ss >> e.v[0] >> e.v[1] >> e.v[2];
            else if (e.kind == 'F')
                ss >> e.v[0] >> e.v[1] >> e.v[2] >> e.v[3];   // week, tow_ms, fix mode, sats
            else if (e.kind == 'K')
            {
                long long mcu0 = 0;
                ss >> e.v[0] >> mcu0 >> e.v[2] >> e.v[3] >> e.v[4] >> e.v[5] >> e.flags;
                e.v[1] = static_cast<double>(mcu0);
                e.flags = e.flags != 0 ? 1 : 0;   // valid
                // the record's seq and restarts, where extract.py wrote them (flag 2)
                long long seq = 0, restarts = 0;
                if (ss >> seq >> restarts)
                {
                    e.v[6] = static_cast<double>(seq);
                    e.v[7] = static_cast<double>(restarts);
                    e.flags |= 2;
                }
            }
            else if (e.kind == 'R')
            {
                std::string h, mark;
                ss >> h;
                e.bytes = unhex(h);
                // extract.py --sbf: a navigation block put at the start (preloaded), filed when a receiver has it
                if (ss >> mark && mark == "preload")
                    e.flags = 1;
            }
            else if (e.kind == 'V')
            {
                // a receiver fix (a sim's PVT): fix mode, satellites, height above the geoid, velocity N E U
                for (int i = 0; i < 6; ++i)
                    ss >> e.v[i];
            }
            else if (e.kind == 'N')
            {
                gps_eph_t eph;
                if (!parseEph(ss, eph))
                    continue;
                e.flags = static_cast<int>(in.neph.size());
                in.neph.push_back(eph);
            }
#if RAW_REPLAY_MOSAIC
            else if (e.kind == 'X')
            {
                // a sim's epoch (sim_events.py): released to the GNSS task at t; its GPS instant on the MCU clock,
                // time of week, week, n; then per satellite sys prn has_pr has_rr, its position, velocity and
                // acceleration (ECEF of the transmit time), pr rr (m, m/s, corrected), C/N0 and an extra Doppler
                // sigma (m/s)
                rr::SimEpoch x;
                long long mcu = 0;
                int n = 0;
                ss >> mcu >> x.tow_s >> x.week >> n;
                x.mcu_us = mcu;
                for (int i = 0; i < n; ++i)
                {
                    rr::SimMeas m;
                    int sys = 0, prn = 0, hp = 0, hr = 0;
                    ss >> sys >> prn >> hp >> hr;
                    for (int k = 0; k < 3; ++k)
                        ss >> m.sat_pos[k];
                    for (int k = 0; k < 3; ++k)
                        ss >> m.sat_vel[k];
                    for (int k = 0; k < 3; ++k)
                        ss >> m.sat_acc[k];
                    ss >> m.pr >> m.rr >> m.cn0_dbhz >> m.rr_sig_add;
                    m.sys = static_cast<uint8_t>(sys);
                    m.prn = static_cast<uint8_t>(prn);
                    m.has_pr = hp != 0;
                    m.has_rr = hr != 0;
                    if (x.n < gnss_raw::RawEpoch::kMaxObs)
                        x.meas[x.n++] = m;
                }
                if (!ss)
                {
                    std::fprintf(stderr, "a bad X line at %lld us\n", static_cast<long long>(e.t_us));
                    continue;
                }
                e.flags = static_cast<int>(in.xep.size());
                in.xep.push_back(x);
            }
#endif
            in.ev.push_back(std::move(e));
        }
    }
    return true;
}

// q (scalar first) = a * b
void quatMul(const double a[4], const double b[4], double o[4])
{
    o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// The GPS -> MCU line the driver's time sync held (GnssTimeSync::mcuAt).
struct ClockLine
{
    bool valid = false;
    double gps0_s = 0, a_s = 0, t_ref_s = 0, drift = 0, latency_s = 0;
    int64_t mcu0_us = 0;
    bool mcuAt(double gps_s, int64_t& out) const
    {
        if (!valid)
            return false;
        const double t = gps_s - gps0_s;
        const double rel = t + (a_s + drift * (t - t_ref_s)) - latency_s;
        out = mcu0_us + static_cast<int64_t>(std::llround(rel * 1.0e6));
        return true;
    }
    // GnssTimeSync::gpsAt: the GPS time of an MCU instant
    double gpsAt(int64_t mcu_us) const
    {
        const double rel = static_cast<double>(mcu_us - mcu0_us) * 1.0e-6;
        return gps0_s + (rel - a_s + drift * t_ref_s + latency_s) / (1.0 + drift);
    }
};

// An emulated FC clock: each call costs fc_unit_us, so the shadow's job
// slices (epoch_slice_us) fall about as they did on the Beetle.
int64_t g_fake_clock_us = 0;
double g_fake_unit_us = 0.0;
int64_t fakeClock()
{
    g_fake_clock_us += static_cast<int64_t>(g_fake_unit_us);
    return g_fake_clock_us;
}

// ---- the diagnosis of one epoch (--diag-from-s/--diag-to-s): the TR_RawGnssEKF.cpp geometry, copied
struct DLos
{
    double rho, rate, u[3], vs[3];
};

void dLineOfSight(const double rs[3], const double vs[3], const double r[3], const double v[3], DLos& L)
{
    const double kC = 299792458.0, kOmegaE = 7.2921151467e-5;
    double d0 = rs[0] - r[0], d1 = rs[1] - r[1], d2 = rs[2] - r[2];
    double rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    double c = 1.0, s = 0.0, rr[3] = {rs[0], rs[1], rs[2]};
    for (int it = 0; it < 2; ++it)
    {
        const double th = kOmegaE * rho / kC;
        const double th2 = th * th;
        c = 1.0 - 0.5 * th2;
        s = th * (1.0 - th2 / 6.0);
        rr[0] = c * rs[0] + s * rs[1];
        rr[1] = -s * rs[0] + c * rs[1];
        rr[2] = rs[2];
        d0 = rr[0] - r[0]; d1 = rr[1] - r[1]; d2 = rr[2] - r[2];
        rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    }
    L.vs[0] = c * vs[0] + s * vs[1];
    L.vs[1] = -s * vs[0] + c * vs[1];
    L.vs[2] = vs[2];
    L.rho = rho;
    L.u[0] = d0 / rho; L.u[1] = d1 / rho; L.u[2] = d2 / rho;
    L.rate = L.u[0] * (L.vs[0] - v[0]) + L.u[1] * (L.vs[1] - v[1]) + L.u[2] * (L.vs[2] - v[2]);
}

double dSatRangeAccel(const RawMeas& m, const DLos& L, const double v[3])
{
    double dvv = 0.0, ua = 0.0;
    for (int k = 0; k < 3; ++k)
    {
        const double dv = L.vs[k] - v[k];
        dvv += dv * dv;
        ua += L.u[k] * m.sat_acc[k];
    }
    return ua + (dvv - L.rate * L.rate) / L.rho;
}

// body -> NED of q (scalar first), and the heading (rotation about down) and tilt (north, east) of b relative to a
void dDcm(const float q[4], double R[3][3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z);     R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y);     R[2][1] = 2 * (y * z + w * x);     R[2][2] = 1 - 2 * (x * x + y * y);
}

void dTurn(const float qa[4], const float qb[4], double out[3])
{
    // e = qb * qa^-1 (NED): small rotation vector, deg (north, east, down)
    const double a0 = qa[0], a1 = -qa[1], a2 = -qa[2], a3 = -qa[3];
    const double b0 = qb[0], b1 = qb[1], b2 = qb[2], b3 = qb[3];
    double e0 = b0 * a0 - b1 * a1 - b2 * a2 - b3 * a3;
    double e1 = b0 * a1 + b1 * a0 + b2 * a3 - b3 * a2;
    double e2 = b0 * a2 - b1 * a3 + b2 * a0 + b3 * a1;
    double e3 = b0 * a3 + b1 * a2 - b2 * a1 + b3 * a0;
    if (e0 < 0) { e0 = -e0; e1 = -e1; e2 = -e2; e3 = -e3; }
    out[0] = 2.0 * e1 * kR2D;
    out[1] = 2.0 * e2 * kR2D;
    out[2] = 2.0 * e3 * kR2D;
}

void diagEpoch(RawGnssShadow& sh, const BuiltEpoch& b, int64_t t_us, uint8_t flags, const Args& a, const float acc[3])
{
    RawGnssEKF<float>& f = sh.ekf_;
    const double t = t_us * 1e-6;
    const double te = b.mcu_us * 1e-6;
    const double age = t - te;
    const double tf = t + a.t_base_s, tef = te + a.t_base_s;
    // the attitude block in NED: heading variance and its covariance with the velocity
    double R[3][3];
    dDcm(f.quat(), R);
    double Pa[3][3], Pav[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            Pa[i][j] = 0.0;
            Pav[i][j] = 0.0;
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
            {
                for (int l = 0; l < 3; ++l)
                    Pa[i][j] += 4.0 * R[i][k] * f.P(6 + k, 6 + l) * R[j][l];
                Pav[i][j] += 2.0 * R[i][k] * f.P(6 + k, 3 + j);
            }
    const double sh_ = std::sqrt(std::max(0.0, Pa[2][2]));
    std::printf("DIAG epoch tow %.3f: tag T%+.3f (GPS time T%+.3f), step T%+.3f, age %.1f ms, %d measurements, "
                "|f| %.1f g, vD %.2f m/s\n",
                b.tow_s, tef, b.tow_s - a.truth_t0_tow, tf, age * 1e3, b.n,
                std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]) / 9.80665, f.velNed()[2]);
    std::printf("DIAG   before: heading sigma %.2f deg, tilt sigma N %.2f E %.2f deg; heading-velocity correlation "
                "N %+.2f E %+.2f D %+.2f\n",
                sh_ * kR2D, std::sqrt(std::max(0.0, Pa[0][0])) * kR2D, std::sqrt(std::max(0.0, Pa[1][1])) * kR2D,
                Pav[2][0] / (sh_ * std::sqrt(f.P(3, 3)) + 1e-30), Pav[2][1] / (sh_ * std::sqrt(f.P(4, 4)) + 1e-30),
                Pav[2][2] / (sh_ * std::sqrt(f.P(5, 5)) + 1e-30));
    // the job, a unit at a time, as onImu would have worked it
    sh.pend_built_[sh.pb_head_] = b;
    sh.npend_built_ = 1;
    sh.startJob(te, flags);
    float q0[4];
    for (int i = 0; i < 4; ++i)
        q0[i] = f.quat()[i];
    for (int guard = 0; sh.job_.active && guard < 400; ++guard)
    {
        const int phase = sh.job_.phase;
        const int k = sh.job_.fused;
        double y_pr = std::nan(""), y_rr = std::nan(""), el = std::nan("");
        if (phase == RawGnssShadow::kJobFuse && k < sh.built_.n)
        {
            const RawMeas& mi = sh.built_.meas[k];
            const double tau = f.rrLag();   // the epoch's (startJob set whether it is in pairs)
            const double ag = t - te;
            float dp_e[3], dv_e[3], dp_l[3], dv_l[3];
            f.historyChange(ag, dv_e, dp_e);
            f.historyChange(ag + tau, dv_l, dp_l);
#if RAW_REPLAY_ANTENNA
            // a known offset: the antenna, as the filter's update back-dates it
            if (f.p_.lever_on)
            {
                RawGnssEKF<float>::LeverEpoch lv;
                f.leverBack(ag, tau, dp_e, dv_e, dv_l, lv);
            }
#endif
            double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
            f.receiverState(r, v, Tm);
            for (int i = 0; i < 3; ++i)
            {
                double sp = 0.0, sv = 0.0;
                for (int j = 0; j < 3; ++j)
                {
                    sp += Tm[j][i] * dp_e[j];
                    sv += Tm[j][i] * dv_e[j];
                }
                r_e[i] = r[i] - sp;
                v_e[i] = v[i] - sv;
            }
            DLos L;
            dLineOfSight(mi.sat_pos, mi.sat_vel, r_e, v_e, L);
            const double b_e = f.clk_b_ - static_cast<double>(f.clk_d_) * ag;
            y_pr = mi.pr - (L.rho + b_e + static_cast<double>(f.isb(mi.sys)));
            double vr[3], v_l[3];
            for (int i = 0; i < 3; ++i)
                vr[i] = Tm[0][i] * f.v_[0] + Tm[1][i] * f.v_[1] + Tm[2][i] * f.v_[2];
            for (int i = 0; i < 3; ++i)
                v_l[i] = vr[i] - (Tm[0][i] * dv_l[0] + Tm[1][i] * dv_l[1] + Tm[2][i] * dv_l[2]);
            DLos Lr = L;
            Lr.rate = 0.0;
            for (int i = 0; i < 3; ++i)
                Lr.rate += L.u[i] * (L.vs[i] - v_l[i]);
            const double pred = tau > 0.0 ? Lr.rate - tau * dSatRangeAccel(mi, Lr, v_l) : Lr.rate;
            y_rr = mi.rr - (pred + static_cast<double>(f.clk_d_));
            // elevation, from the NED line of sight
            const double ud = Tm[2][0] * L.u[0] + Tm[2][1] * L.u[1] + Tm[2][2] * L.u[2];
            el = std::asin(std::max(-1.0, std::min(1.0, -ud))) * kR2D;
            // the line of sight in NED, for the record
            const double un = Tm[0][0] * L.u[0] + Tm[0][1] * L.u[1] + Tm[0][2] * L.u[2];
            const double ue = Tm[1][0] * L.u[0] + Tm[1][1] * L.u[1] + Tm[1][2] * L.u[2];
            std::printf("DIAG   sat %c%02d el %4.1f az %5.1f: pr innovation %+9.2f m (sigma %.1f+%.1f), Doppler "
                        "innovation %+7.2f m/s (sigma %.2f)",
                        mi.sys == 0 ? 'G' : (mi.sys == 1 ? 'E' : 'C'), mi.prn, el, std::atan2(ue, un) * kR2D, y_pr,
                        mi.sigma_pr, mi.sigma_pr_corr, y_rr, mi.sigma_rr);
            // and, for a check against the truth, the measurement and the geometry (ECEF)
            std::fprintf(stderr, "DIAGDATA %.3f %d %d %.4f %.5f %.4f %.4f %.4f %.6f %.6f %.6f %.4f %.4f %.4f %.5f %.5f %.5f "
                                 "%.3f %.3f %.3f %.3f\n",
                         b.tow_s, mi.sys, mi.prn, mi.pr, mi.rr, mi.sat_pos[0], mi.sat_pos[1], mi.sat_pos[2],
                         mi.sat_vel[0], mi.sat_vel[1], mi.sat_vel[2], r_e[0], r_e[1], r_e[2], v_l[0], v_l[1], v_l[2],
                         y_pr, y_rr, static_cast<double>(mi.sigma_rr), static_cast<double>(mi.sigma_pr));
        }
        const int used_pr0 = sh.job_.rs.used_pr, used_rr0 = sh.job_.rs.used_rr;
        float qb[4], vb[3];
        for (int i = 0; i < 4; ++i)
            qb[i] = f.quat()[i];
        for (int i = 0; i < 3; ++i)
            vb[i] = f.velNed()[i];
        sh.jobUnit(t);
        double turn[3];
        dTurn(qb, f.quat(), turn);
        if (phase == RawGnssShadow::kJobFuse && k < sh.built_.n)
            std::printf(" -> used pr %d rr %d; heading %+.3f deg, tilt N %+.3f E %+.3f deg, velocity N %+.3f E %+.3f D "
                        "%+.3f m/s\n",
                        sh.job_.rs.used_pr - used_pr0, sh.job_.rs.used_rr - used_rr0, turn[2], turn[0], turn[1],
                        f.velNed()[0] - vb[0], f.velNed()[1] - vb[1], f.velNed()[2] - vb[2]);
        else if (std::fabs(turn[0]) + std::fabs(turn[1]) + std::fabs(turn[2]) > 1e-4)
            std::printf("DIAG   unit %d: heading %+.3f deg, tilt N %+.3f E %+.3f deg\n", phase, turn[2], turn[0],
                        turn[1]);
    }
    double tot[3];
    dTurn(q0, f.quat(), tot);
    std::printf("DIAG   epoch total: used pr %d rr %d, rejected pr %d rr %d; heading %+.3f deg, tilt N %+.3f E "
                "%+.3f deg\n",
                sh.job_.rs.used_pr, sh.job_.rs.used_rr, sh.job_.rs.rejected_pr, sh.job_.rs.rejected_rr, tot[2],
                tot[0], tot[1]);
}

struct GuardStats
{
    int epochs = 0, blocked_pr = 0, blocked_dop = 0, refused = 0, tightened = 0, rejected = 0, majority = 0;
    std::vector<double> majority_tow;   // the epochs the majority check dropped (time of week, s)
};

// --compare-sequential: one boost epoch's two verdicts, from the same state
struct ScreenCmp
{
    double tow = 0.0, t_flight = 0.0, g = 0.0;
    bool apogee = false;          // the apogee call already made (the firmware never screens then)
    bool fw_screened = false, fw_drop = false;
    int fw_rr = 0, fw_refused = 0;
    int seq_used = 0, seq_rej = 0;
    bool seq_drop = false;
    int meas = 0, seq_used_pr = 0, seq_rej_pr = 0;
    int fw_dry_used = -1, fw_dry_rej = 0;   // the firmware's own scratch-copy counts (-1: not worked through)
};

// An epoch worked unit by unit with the boost safeguards (the epoch held out of the step that would have applied it)
void guardedEpoch(RawGnssShadow& sh, const BuiltEpoch& b, int64_t t_us, uint8_t flags, const Args& a, GuardStats& gs)
{
    RawGnssEKF<float>& f = sh.ekf_;
    const double t = t_us * 1e-6;
    const double te = b.mcu_us * 1e-6;
    const double gpk = static_cast<double>(f.g_peak_);
    const bool boost = gpk > a.guard_g;
    const RawGnssEKFParams saved = f.p_;
    bool block = false;
    bool hold_only = false;   // (a3vh): held from the attitude, never dropped
    if (boost)
    {
        gs.epochs++;
        int npr = 0;
        for (int i = 0; i < b.n; ++i)
            npr += b.meas[i].has_pr ? 1 : 0;
        if (a.min_pr > 0 && npr < a.min_pr)
        {
            block = true;
            gs.blocked_pr++;
        }
        if ((a.dop_rms_mps > 0.0 || a.dv_max_mps > 0.0 || a.dv_sigma > 0.0 || a.dv_vert_max > 0.0 ||
             a.dv_vert_hold > 0.0) && !block)
        {
            // the epoch's Dopplers against the filter, a common velocity and clock rate fitted to them: their
            // spread about the fit (the Dopplers' instant as the epoch's job will take it, RawGnssEKF::rrLag)
            const double lag =
                f.p_.rr_lag_s + (b.rr_paired != 0 && f.p_.rr_pair_lag_s > 0.0 ? f.p_.rr_pair_lag_s : 0.0);
            const double tau = lag > 0.0 ? lag : 0.0;
            const double ag = t - te;
            float dp_e[3], dv_e[3], dp_l[3], dv_l[3];
            f.historyChange(ag, dv_e, dp_e);
            f.historyChange(ag + tau, dv_l, dp_l);
#if RAW_REPLAY_ANTENNA
            // a known offset: the antenna's, as the filter predicts them (its turning is not a velocity error)
            if (f.p_.lever_on)
            {
                RawGnssEKF<float>::LeverEpoch lv;
                f.leverBack(ag, tau, dp_e, dv_e, dv_l, lv);
            }
#endif
            double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
            f.receiverState(r, v, Tm);
            for (int i = 0; i < 3; ++i)
            {
                double sp = 0.0, sv = 0.0;
                for (int j = 0; j < 3; ++j)
                {
                    sp += Tm[j][i] * dp_e[j];
                    sv += Tm[j][i] * dv_e[j];
                }
                r_e[i] = r[i] - sp;
                v_e[i] = v[i] - sv;
            }
            double vr[3], v_l[3];
            for (int i = 0; i < 3; ++i)
                vr[i] = Tm[0][i] * f.v_[0] + Tm[1][i] * f.v_[1] + Tm[2][i] * f.v_[2];
            for (int i = 0; i < 3; ++i)
                v_l[i] = vr[i] - (Tm[0][i] * dv_l[0] + Tm[1][i] * dv_l[1] + Tm[2][i] * dv_l[2]);
            double HtH[4][4] = {}, Hty[4] = {}, ys[RawEpoch::kMaxObs], H[RawEpoch::kMaxObs][4];
            int n = 0;
            for (int i = 0; i < b.n && n < RawEpoch::kMaxObs; ++i)
            {
                const RawMeas& mi = b.meas[i];
                if (!mi.has_rr)
                    continue;
                DLos L;
                dLineOfSight(mi.sat_pos, mi.sat_vel, r_e, v_e, L);
                DLos Lr = L;
                Lr.rate = 0.0;
                for (int k = 0; k < 3; ++k)
                    Lr.rate += L.u[k] * (L.vs[k] - v_l[k]);
                const double pred = tau > 0.0 ? Lr.rate - tau * dSatRangeAccel(mi, Lr, v_l) : Lr.rate;
                ys[n] = mi.rr - (pred + static_cast<double>(f.clk_d_));
                H[n][0] = -L.u[0];
                H[n][1] = -L.u[1];
                H[n][2] = -L.u[2];
                H[n][3] = 1.0;
                for (int j = 0; j < 4; ++j)
                {
                    Hty[j] += H[n][j] * ys[n];
                    for (int k = 0; k < 4; ++k)
                        HtH[j][k] += H[n][j] * H[n][k];
                }
                n++;
            }
            if (n >= (a.dop_rms_mps > 0.0 ? 5 : 4))
            {
                // solve the 4x4 normal equations (Gauss-Jordan)
                double M[4][5];
                for (int j = 0; j < 4; ++j)
                {
                    for (int k = 0; k < 4; ++k)
                        M[j][k] = HtH[j][k];
                    M[j][4] = Hty[j];
                }
                bool ok = true;
                for (int c = 0; c < 4 && ok; ++c)
                {
                    int piv = c;
                    for (int r2 = c + 1; r2 < 4; ++r2)
                        if (std::fabs(M[r2][c]) > std::fabs(M[piv][c]))
                            piv = r2;
                    if (std::fabs(M[piv][c]) < 1e-9)
                    {
                        ok = false;
                        break;
                    }
                    for (int k = 0; k < 5; ++k)
                        std::swap(M[c][k], M[piv][k]);
                    for (int r2 = 0; r2 < 4; ++r2)
                    {
                        if (r2 == c)
                            continue;
                        const double fct = M[r2][c] / M[c][c];
                        for (int k = 0; k < 5; ++k)
                            M[r2][k] -= fct * M[c][k];
                    }
                }
                if (ok)
                {
                    double x[4];
                    for (int j = 0; j < 4; ++j)
                        x[j] = M[j][4] / M[j][j];
                    double ss = 0.0;
                    for (int i = 0; i < n; ++i)
                    {
                        double e = ys[i];
                        for (int j = 0; j < 4; ++j)
                            e -= H[i][j] * x[j];
                        ss += e * e;
                    }
                    const double rms = std::sqrt(ss / std::max(1, n - 4));
                    const double dv = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
                    // the threshold: dv_max, or dv_sigma of the filter's own velocity 1-sigma when that is larger
                    // (a filter that has drifted in boost, e.g. on a magnetometer 40 deg off, is not a bad epoch)
                    double dv_thr = a.dv_max_mps;
                    if (a.dv_sigma > 0.0)
                    {
                        const double sv = std::sqrt(std::max(0.0, static_cast<double>(f.P(3, 3) + f.P(4, 4) +
                                                                                       f.P(5, 5))));
                        dv_thr = std::max(dv_thr, a.dv_sigma * sv);
                    }
                    // the fitted correction's down component (Tm's rows: N, E, D in ECEF)
                    const double dv_down = Tm[2][0] * x[0] + Tm[2][1] * x[1] + Tm[2][2] * x[2];
                    if (std::getenv("RAW_REPLAY_DVLOG"))
                        std::fprintf(stderr, "DV tow %.1f n %d rms %.2f dv %.2f down %+.2f\n", b.tow_s, n, rms, dv, dv_down);
                    if ((a.dop_rms_mps > 0.0 && rms > a.dop_rms_mps) || (dv_thr > 0.0 && dv > dv_thr) ||
                        (a.dv_vert_max > 0.0 && std::fabs(dv_down) > a.dv_vert_max))
                    {
                        block = true;
                        gs.blocked_dop++;
                    }
                    else if (a.dv_vert_hold > 0.0 && std::fabs(dv_down) > a.dv_vert_hold)
                    {
                        hold_only = true;     // the attitude held out, the rest fused
                        gs.blocked_dop++;
                    }
                }
            }
            else if (n > 0 && !a.dop_thin_pass)
            {
                // too few to judge their agreement: held out as well (--guard-dop-thin-pass: passed)
                block = true;
                gs.blocked_dop++;
            }
        }
        if (hold_only)
            block = true;
        else if (block && a.reject_epoch)
        {
            // (a4) the whole epoch dropped: nothing of it reaches the filter. The IMU peak restarts as it does when
            // an epoch is used (startJob takes it): "boost" is the peak since the last epoch, used or not
            f.takeGPeak();
            gs.rejected++;
            return;
        }
        if (block)
        {
            // GNSS keeps the velocity, position and clock; the attitude gets cos^4(pitch) of it: nothing near vertical
            f.p_.att_gate_cos4 = true;
            f.p_.att_gate_vertical = false;
        }
        if (a.gate_chi2 > 0.0 && gpk > a.tight_g)
        {
            f.p_.gate_chi2 = a.gate_chi2;
            gs.tightened++;
        }
    }
    // (a6) the epoch judged by its own measurements after the gate has seen them: the filter as it was before it,
    // kept in case most of its range rates are refused
    std::unique_ptr<RawGnssEKF<float>> keep_epoch;
    if (boost && a.majority)
        keep_epoch.reset(new RawGnssEKF<float>(f));
    sh.pend_built_[sh.pb_head_] = b;
    sh.npend_built_ = 1;
    sh.startJob(te, flags);
    // the heading budget for this epoch (b)
    double budget = 1e9;
    if (boost && a.step_deg > 0.0)
        budget = a.step_deg;
    if (boost && a.step_sigma > 0.0)
    {
        float att[3];
        f.attSigmaNed(att);
        budget = std::min(budget, a.step_sigma * att[2] * kR2D);
    }
    float q0[4];
    for (int i = 0; i < 4; ++i)
        q0[i] = f.quat()[i];
    for (int guard = 0; sh.job_.active && guard < 400; ++guard)
    {
        const int phase = sh.job_.phase;
        if (phase == RawGnssShadow::kJobFuse && budget < 1e8)
        {
            std::unique_ptr<RawGnssEKF<float>> keep(new RawGnssEKF<float>(f));
            const RawGnssStats rs_keep = sh.job_.rs;
            sh.jobUnit(t);
            double tot[3];
            dTurn(q0, f.quat(), tot);
            if (std::fabs(tot[2]) > budget)
            {
                // this satellite would turn the heading past the epoch's budget: refused, as if never fused
                const int fused = sh.job_.fused;
                const int ph = sh.job_.phase;
                f = *keep;
                sh.job_.rs = rs_keep;
                sh.job_.fused = fused;
                sh.job_.phase = ph;
                gs.refused++;
            }
        }
        else
        {
            sh.jobUnit(t);
        }
    }
    if (keep_epoch && (sh.job_.rs.rejected_rr > sh.job_.rs.used_rr ||
                       (a.majority_half && sh.job_.rs.rejected_rr > 0 && sh.job_.rs.rejected_rr >= sh.job_.rs.used_rr)))
    {
        // (a6) more of its range rates refused than used: a receiver that lost track (c30's T+1 had 4 of 7 refused
        // and the 3 it let through turned the heading 23 deg); the epoch is dropped whole, the IMU peak restarted
        f = *keep_epoch;
        f.takeGPeak();
        gs.majority++;
        gs.majority_tow.push_back(b.tow_s);
    }
    // the gates and the innovation gate back as the configuration (and act's launch edge) left them
    f.p_.att_gate_cos4 = saved.att_gate_cos4;
    f.p_.att_gate_vertical = saved.att_gate_vertical;
    f.p_.gate_chi2 = saved.gate_chi2;
}

#if RAW_REPLAY_DIGEST
// ---- --digest: what a step left, hashed field by field. The lists below are
// fixed (bitident.sh builds this file frozen at the commit it names):
// every field is named, so a tree that only adds fields hashes the same, and
// any change to what the Beetle's filter does with today's fields does not.
struct Fnv
{
    uint64_t h = 1469598103934665603ULL;   // FNV-1a, 64-bit
    void add(const void* p, size_t n)
    {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i)
        {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    }
    template <typename T>
    void put(const T& v)
    {
        add(&v, sizeof(v));
    }
};

// tests_cpp/test_raw_gnss_shadow.cpp's stateHash: the filter's state and its whole covariance
uint64_t stateHash(const RawGnssEKF<float>& f)
{
    Fnv h;
    h.add(f.lla(), 3 * sizeof(double));
    h.add(f.velNed(), 3 * sizeof(float));
    h.add(f.quat(), 4 * sizeof(float));
    h.add(f.accelBias(), 3 * sizeof(float));
    h.add(f.gyroBias(), 3 * sizeof(float));
    const double cb = f.clockBias();
    const float cd = f.clockDrift();
    h.put(cb);
    h.put(cd);
    for (int i = 0; i < RawGnssEKF<float>::N; ++i)
        for (int j = 0; j < RawGnssEKF<float>::N; ++j)
        {
            const float p = f.P(i, j);
            h.put(p);
        }
    return h.h;
}

// the navigation state the flight loop flies on (ShadowNav), every field
uint64_t navHash(const ShadowNav& n)
{
    Fnv h;
    h.put(n.time_us);
    h.put(n.lla);
    h.put(n.vel_ned);
    h.put(n.quat);
    h.put(n.accel_bias);
    h.put(n.gyro_bias);
    h.put(n.var);
    h.put(n.running);
    h.put(n.gnss_aided);
    h.put(n.baro_aided);
    h.put(n.in_flight);
    h.put(n.epochs);
    return h.h;
}

// The counters (RawGnssShadow::Stats), not the CPU times: those are the
// clock's, and the harness has none unless --fc-unit-us emulates one.
uint64_t statsHash(const RawGnssShadow::Stats& s)
{
    Fnv h;
    // ticks, starts and stops; epochs seen, idle, stale and applied; ephemerides
    h.put(s.imu);
    h.put(s.imu_repeats);
    h.put(s.starts);
    h.put(s.stops);
    h.put(s.last_stop);
    h.put(s.epochs);
    h.put(s.epochs_untimed);
    h.put(s.epochs_idle);
    h.put(s.epochs_stale);
    h.put(s.epochs_applied);
    h.put(s.eph_added);
    h.put(s.eph_bad);
    h.put(s.resyncs);
    h.put(s.step_n);
    // the last epoch applied: what it used and rejected of each kind, its clock steps, its screens
    const RawGnssStats& l = s.last;
    h.put(l.used_pr);
    h.put(l.used_rr);
    h.put(l.rejected_pr);
    h.put(l.rejected_rr);
    h.put(l.clock_jump_ms);
    h.put(l.clock_step_m);
    h.put(l.drift_step_mps);
    h.put(l.nis_max);
    h.put(l.used_cr);
    h.put(l.cr_built);
    h.put(l.cr_unmatched);
    h.put(l.cr_in);
    h.put(l.cr_half);
    h.put(l.cr_repaired);
    h.put(l.cr_other);
    h.put(l.cr_too_few);
    h.put(l.cr_rms_mps);
    h.put(l.cr_fit_rms_mps);
    h.put(l.cr_dv_mps);
    h.put(l.cr_roll_rev_s);
    h.put(l.cr_common_mps);
    h.put(l.screen_rr);
    h.put(l.screen_refused);
    h.put(l.screen_seq_used);
    h.put(l.screen_seq_refused);
    h.put(l.screen_dropped);
    h.put(l.screen_fit_n);
    h.put(l.screen_fit_out);
    h.put(l.screen_dv_down_mps);
    h.put(s.last_age_s);
    h.put(s.last_tow_ms);
    h.put(s.last_meas);
    h.put(s.last_pad_corrected);
    h.put(s.tracked);
    h.put(s.tracked_no_eph);
    h.put(s.survey_epochs);
    h.put(s.iono_sets);
    // the physics checks, the receiver's fixes
    h.put(s.last_sats_jumped);
    h.put(s.sats_jumped);
    h.put(s.fixes);
    h.put(s.fixes_untimed);
    h.put(s.fixes_dropped);
    // levelling, the barometer, zero velocity, the magnetometer
    h.put(s.levels);
    h.put(s.levels_held);
    h.put(s.baro);
    h.put(s.baro_fused);
    h.put(s.baro_held);
    h.put(s.baro_offset_m);
    h.put(s.baro_offset_frozen);
    h.put(s.zupts);
    h.put(s.pad_zupts);
    h.put(s.mags);
    h.put(s.mag_fused);
    h.put(s.mag_uncal);
    h.put(s.mag_suspect);
    h.put(s.mag_dip);
    h.put(s.mag_dip_err_deg);
    // restores, bridged IMU holes, pad contexts
    h.put(s.restores);
    h.put(s.gaps_bridged);
    h.put(s.pad_contexts);
    // the heading from thrust, and act's launch edge, gates and hand-offs
    h.put(s.heading_seeds);
    h.put(s.heading_flights);
    h.put(s.heading_epochs);
    h.put(s.heading_vel_fails);
    h.put(s.heading_unset_launches);
    h.put(s.heading_launch_sigma_deg);
    h.put(s.heading_launch_mag_age_s);
    h.put(s.heading_act_launches);
    h.put(s.heading_mag_launches);
    h.put(s.heading_launch_prior_deg);
    h.put(s.heading_launch_mag);
    h.put(s.heading_gate_apogee);
    h.put(s.heading_gate_apogee_s);
    h.put(s.heading_handoffs);
    h.put(s.heading_rivals);
    h.put(s.heading_turn_deg);
    h.put(s.heading_handoff_s);
    const HeadingFit::Result& r = s.heading_last;
    h.put(r.ok);
    h.put(r.psi);
    h.put(r.sigma);
    h.put(r.rival);
    h.put(r.bias);
    h.put(r.n);
    h.put(r.n_cut);
    h.put(r.rms);
    h.put(r.evals);
    // the carrier range rates
    h.put(s.cr_epochs);
    h.put(s.cr_too_few);
    h.put(s.cr_built);
    h.put(s.cr_unmatched);
    h.put(s.cr_in);
    h.put(s.cr_half);
    h.put(s.cr_repaired);
    h.put(s.cr_other);
    h.put(s.cr_used);
    h.put(s.cr_last_rms_mps);
    h.put(s.cr_last_fit_rms_mps);
    h.put(s.cr_last_dv_mps);
    h.put(s.cr_last_common_mps);
    h.put(s.cr_last_roll_rev_s);
    // the boost screen, the vertical check and its latch
    h.put(s.boost_screened);
    h.put(s.boost_dropped);
    h.put(s.boost_dropped_dry);
    h.put(s.boost_last_rr);
    h.put(s.boost_last_refused);
    h.put(s.boost_last_dry_used);
    h.put(s.boost_last_dry_refused);
    h.put(s.boost_last_g);
    h.put(s.vert_checked);
    h.put(s.vert_dropped);
    h.put(s.vert_last_dv_down);
    h.put(s.vert_worst_dv_down);
    h.put(s.vert_latched);
    return h.h;
}

std::string hex64(uint64_t v)
{
    char b[17];
    std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(v));
    return b;
}

// main.cpp's raw task takes a PAD_CONTEXT_MSG record at most this often
// (config::PAD_CONTEXT_PERIOD_MS), stamped with its FC micros
constexpr int64_t kPadContextPeriodUs = 20000;

// One step's line: its count and IMU time, then the hashes of the navigation
// state, the RAW_GNSS_EKF_MSG (0x96) and RAW_GNSS_ATT_MSG (0x9B) records, the
// HEADING_FIT_MSG records and the PAD_CONTEXT_MSG record it filled ("-": none
// this step, as main.cpp's raw task would have found), the counters, and the
// filter's whole state. Taking the records changes nothing the filter does:
// they are its log's. The pad context is taken as the raw task takes it, at
// the step's IMU time (t_us; pcx_t0_us: when the last one was taken).
void writeDigest(FILE* out, RawGnssShadow& sh, uint32_t time_us, int64_t t_us, int64_t& pcx_t0_us)
{
    ShadowNav nav;
    sh.navState(nav);
    std::string rec = "-", att = "-", hdg = "-", pcx = "-";
    RawGnssEkfData r;
    if (sh.takeLog(r))
    {
        Fnv h;
        h.put(r);
        rec = hex64(h.h);
    }
    RawGnssAttData ra;
    if (sh.takeAttLog(ra))
    {
        Fnv h;
        h.put(ra);
        att = hex64(h.h);
    }
    HeadingFitData hf;
    Fnv hh;
    int nh = 0;
    while (sh.takeHeadingLog(hf))
    {
        hh.put(hf);
        nh++;
    }
    if (nh > 0)
        hdg = hex64(hh.h);
    if (t_us - pcx_t0_us >= kPadContextPeriodUs)
    {
        uint8_t buf[MAX_PAYLOAD];
        size_t len = 0;
        if (sh.takePadContext(buf, sizeof(buf), len, static_cast<uint32_t>(t_us)))
        {
            pcx_t0_us = t_us;
            Fnv h;
            h.put(static_cast<uint32_t>(len));
            h.add(buf, len);
            pcx = hex64(h.h);
        }
    }
    std::fprintf(out, "%u %u %s %s %s %s %s %s %s\n", sh.stats().step_n, time_us, hex64(navHash(nav)).c_str(),
                 rec.c_str(), att.c_str(), hdg.c_str(), pcx.c_str(), hex64(statsHash(sh.stats())).c_str(),
                 hex64(stateHash(sh.filter())).c_str());
}
#endif

}  // namespace

int main(int argc, char** argv)
{
    Args a;
    if (!parseArgs(argc, argv, a))
    {
        usage();
        return 2;
    }
    const bool mosaic = a.receiver == "mosaic";
    if (a.receiver != "lc86g" && !mosaic)
    {
        std::fprintf(stderr, "--receiver is lc86g or mosaic, not %s\n", a.receiver.c_str());
        return 2;
    }
    // the cheaper epoch build as the firmware allows it: on the mosaic-only image alone (its CMakeLists refuses
    // TR_BENCH_RAW_BUILD_LITE elsewhere), so never on the LC86G's profile
    if (a.build_lite && !mosaic)
    {
        std::fprintf(stderr, "--build-lite is for the mosaic-G5 (--receiver mosaic) alone, as TR_BENCH_RAW_BUILD_LITE "
                             "is for the mosaic-only image\n");
        return 2;
    }
    if (!a.flight_state.empty() && a.flight_state != "log" && a.flight_state != "quorum")
    {
        std::fprintf(stderr, "--flight-state is log or quorum, not %s\n", a.flight_state.c_str());
        return 2;
    }
#if !RAW_REPLAY_MOSAIC
    if (mosaic || a.cold_start || a.unaided || !a.rinex.empty() || a.quorum || a.sat_cap >= 0 || a.fuse_ms > 0.0 ||
        a.share_los >= 0 || a.flight_state == "quorum" || a.time_tag_from_boot || a.assume_finetime ||
        a.seed_heading_error_deg != 0.0 || !a.truth_nav.empty())
    {
        std::fprintf(stderr, "this build has no mosaic-G5 path (RAW_REPLAY_MOSAIC=OFF): --receiver mosaic, "
                             "--cold-start, --unaided, --rinex, --quorum, --sat-cap, --fuse-ms, --share-los, "
                             "--flight-state quorum, --time-tag-from-boot, --assume-finetime, "
                             "--seed-heading-error-deg and --truth-nav need it\n");
        return 2;
    }
#endif
#if !(RAW_REPLAY_MOSAIC && RAW_REPLAY_DOPAVG)
    if (a.rr_pair_avg > 0 || a.rr_pair_max_diff >= 0.0 || a.rr_pair_sigma >= 0.0 || a.sim_fuse_grid)
    {
        std::fprintf(stderr, "this build has no Dopplers in pairs (RAW_REPLAY_MOSAIC and RAW_REPLAY_DOPAVG): "
                             "--rr-pair-avg, --rr-pair-max-diff, --rr-pair-sigma and --sim-fuse-grid need it\n");
        return 2;
    }
#endif
    Input in;
    if (!readInput(a.in, in) || (!a.cold_start && !mosaic && in.s.size() < 20))
    {
        std::fprintf(stderr, "cannot read %s (or no start state)\n", a.in.c_str());
        return 1;
    }
    // the antenna's offset the flight's filter flew with (extract.py: the log's PCX_ANT record, body FRD, m), where
    // --antenna-m gives none: a replay of a flight predicts at the antenna as its filter did
    const bool ant_given = a.ant_known || a.ant_none;
    if (!ant_given && in.meta.count("antenna_m"))
    {
        if (std::sscanf(in.meta["antenna_m"].c_str(), "%lf %lf %lf", &a.ant_m[0], &a.ant_m[1], &a.ant_m[2]) != 3)
        {
            std::fprintf(stderr, "cannot read META antenna_m: %s\n", in.meta["antenna_m"].c_str());
            return 1;
        }
        a.ant_known = true;
        std::fprintf(stderr, "the antenna's offset the flight's filter flew with, from the log: %.3f, %.3f, %.3f m "
                             "(body FRD; --antenna-m none replays it unknown)\n", a.ant_m[0], a.ant_m[1], a.ant_m[2]);
    }
    // a setting on the flight that its filter did not fly with (META antenna_m_set): the filter was on an earlier
    // offset (META antenna_m, flown above) or on none (replayed unknown, as it flew); only named
    if (!ant_given && in.meta.count("antenna_m_set"))
    {
        double m[3];
        if (std::sscanf(in.meta["antenna_m_set"].c_str(), "%lf %lf %lf", &m[0], &m[1], &m[2]) != 3)
        {
            std::fprintf(stderr, "cannot read META antenna_m_set: %s\n", in.meta["antenna_m_set"].c_str());
            return 1;
        }
        std::fprintf(stderr, "the flight had an antenna offset set (%.3f, %.3f, %.3f m, body FRD) that its navigation "
                             "filter did not fly with: %s (--antenna-m %.3f,%.3f,%.3f applies the setting)\n",
                     m[0], m[1], m[2],
                     in.meta.count("antenna_m") ? "the replay flies the offset the filter flew"
                                                : "replayed unknown, as it flew",
                     m[0], m[1], m[2]);
    }
    // a mosaic log has no record of the filter's own (no 0x96 yet): it starts on the pad
    const bool cold = a.cold_start || in.s.size() < 20;
#if RAW_REPLAY_MOSAIC
    // a sim's epochs (X) are built already: the synthesized faults that act on decoded epochs (R) never reach them,
    // and a run given them would fly as a fault-free one with nothing to say so -- the sim writes its faults in
    if (!in.xep.empty() && (a.given.count("--cut-from-s") || a.given.count("--cut-to-s") ||
                            a.given.count("--corrupt-at-s") || a.given.count("--relock-min-lock-ms")))
    {
        std::fprintf(stderr, "%s has X lines (a sim's epochs, its faults in them): --cut-from-s, --cut-to-s, "
                             "--corrupt-at-s and --relock-min-lock-ms act on decoded epochs only\n",
                     a.in.c_str());
        return 2;
    }
#endif
    if (cold && !a.cold_start)
        std::fprintf(stderr, "no start state in %s: a cold start on the pad (--cold-start)\n", a.in.c_str());
    const bool have_pnav = !cold && in.pnav.size() >= 35;   // PNAV: extract.py's 35 values (below)
    const bool have_pcov = !cold && in.pcov.size() == 190;
    // The mosaic-only build flies the heading at the firmware's defaults (TR_HEADING_FIT 2 with the navigation
    // filter, the gate RAW_HEADING_GATE, the Config's prior, hand-off and tilt floor), and the GNSS task hands its
    // epochs over when built (no on-board age to match yet): ~30 ms after they arrive (E0's estimate, until the
    // bench measures the P4's build)
    if (mosaic)
    {
        a.fw_defaults = true;
        if (!a.given.count("--epoch-delay-ms"))
            a.epoch_delay_ms = 30.0;
        a.match_ages = false;
    }
    // the flight's time from the events file where it says (extract.py --sbf, a sim's), unless given
    if ((mosaic || cold) && !a.given.count("--t-base-s") && in.meta.count("t_base_s"))
        a.t_base_s = std::atof(in.meta["t_base_s"].c_str());
    if (a.fw_defaults && !a.given.count("--heading-mode"))
        a.heading_mode = 2;   // act: TR_HEADING_FIT = 2 where the raw filter is primary

    // ---- the configuration, as startRawShadow() builds it, with the bench flags
    RawGnssShadow::Config cfg;
#if !RAW_REPLAY_OLD
#if RAW_REPLAY_MOSAIC
    GnssReceiverProfile rx = mosaic ? mosaicG5Profile() : lc86gProfile();
    // the mosaic build's bench macros, on the profile before it is applied (main.cpp startRawShadow)
    if (a.fuse_ms > 0.0)
        rx.fuse_epoch_period_s = a.fuse_ms * 1.0e-3;   // TR_BENCH_RAW_FUSE_MS
    if (a.sat_cap >= 0)
        rx.sat_cap = a.sat_cap;                         // TR_BENCH_RAW_SAT_CAP
#if RAW_REPLAY_BUILD_LITE
    if (a.build_lite)
        rx.build_lite = true;                           // TR_BENCH_RAW_BUILD_LITE
#else
    if (a.build_lite)
    {
        std::fprintf(stderr, "this build has no cheaper epoch build (RAW_REPLAY_BUILD_LITE=OFF): --build-lite needs it\n");
        return 2;
    }
#endif
#if RAW_REPLAY_SHARELOS
    if (a.share_los >= 0)
    {
        rx.share_los = a.share_los >= 1;                // TR_BENCH_RAW_SHARE_LOS
        rx.rr_float = a.share_los >= 2;
    }
#else
    if (a.share_los >= 0)
        std::fprintf(stderr, "WARNING: this build has no shared epoch geometry (RAW_REPLAY_SHARELOS=OFF): --share-los "
                             "is ignored\n");
#endif
#if RAW_REPLAY_DOPAVG
    if (a.rr_pair_avg >= 0)
        rx.rr_pair_avg = a.rr_pair_avg != 0;            // TR_BENCH_RAW_DOP_AVG
    if (a.rr_pair_max_diff >= 0.0)
        rx.rr_pair_max_diff_mps = a.rr_pair_max_diff;
    if (rx.rr_pair_avg && !dopplerPairsFit(rx))
    {
        std::fprintf(stderr, "--rr-pair-avg 1 needs a fused grid of two receiver epochs (the mosaic's --fuse-ms 100), "
                             "not %.0f ms over %.0f ms epochs\n",
                     rx.fuse_epoch_period_s * 1000.0, rx.raw_epoch_period_s * 1000.0);
        return 2;
    }
#endif
#if RAW_REPLAY_CAPGOV
    if (a.cap_max >= 0)
        rx.sat_cap_max = a.cap_max;                     // TR_BENCH_RAW_CAP_MAX
    if (a.slice_us > 0.0)
        rx.epoch_slice_us = a.slice_us;                 // TR_BENCH_RAW_SLICE_US
#else
    if (a.cap_max >= 0 || a.slice_us > 0.0)
        std::fprintf(stderr, "WARNING: this build has no cap governor or profile slice (RAW_REPLAY_CAPGOV=OFF): "
                             "--cap-max and --slice-us are ignored\n");
#endif
#else
    GnssReceiverProfile rx = lc86gProfile();
#endif
    if (a.carrier >= 0)
        rx.cr_mode = static_cast<uint8_t>(a.carrier);
    if (a.half_cycle_repair >= 0)
        rx.cr_half_cycle = a.half_cycle_repair ? kHalfCycleRepair : kHalfCycleReject;
    if (a.rr_dyn_sigma >= 0.0)
        rx.rr_dyn_sigma = a.rr_dyn_sigma;
#if RAW_REPLAY_DYNMEM
    if (a.rr_dyn_tau_s >= 0.0)
        rx.rr_dyn_tau_s = a.rr_dyn_tau_s;
#else
    if (a.rr_dyn_tau_s >= 0.0)
        std::fprintf(stderr, "WARNING: this build has no Doppler settling (RAW_REPLAY_DYNMEM=OFF): --rr-dyn-tau-s is "
                             "ignored\n");
#endif
    applyReceiverProfile(rx, cfg);
#if RAW_REPLAY_MOSAIC_SAFEGUARDS
    // the mosaic-only image's flight safeguards, where startRawShadow() sets them: after the profile, before any
    // bench flag -- so --receiver mosaic flies the image's settings, and the --dv-*, --hdg-obs-* and --rot-* flags
    // below change them for an experiment (0 turns one off: today's design before them, and its twin)
    if (mosaic)
        applyMosaicFlightSafeguards(cfg);
#else
    // a build without them (an older tree, or one of RAW_REPLAY_DVCHECK, HDGOBS and ROT off) flies none of the
    // three: not the image
    if (mosaic)
        std::fprintf(stderr, "WARNING: this build does not apply the mosaic image's flight safeguards "
                             "(RAW_REPLAY_MOSAIC_SAFEGUARDS=OFF): --receiver mosaic replays with the Doppler change "
                             "check, the heading share and the turning body's allowance off unless set by flag\n");
#endif
    // an experiment on the Doppler's instant: the filter's rr_lag_s in place of the profile's (offline only)
    if (a.rr_lag_ms >= 0.0)
        cfg.ekf.rr_lag_s = a.rr_lag_ms * 1.0e-3;
    // an experiment on the filter's own parameter: the Doppler's refusal count before it is let back in (the
    // profile sets it from reject_persist_s; a field of the fc/v9-raw-primary tree, as the mosaic path's)
#if RAW_REPLAY_MOSAIC
    if (a.reject_persist_rr > 0)
        cfg.ekf.reject_persist_rr = std::min(255, a.reject_persist_rr);
#else
    if (a.reject_persist_rr > 0)
        std::fprintf(stderr, "WARNING: this build has no reject_persist_rr (RAW_REPLAY_MOSAIC=OFF): "
                             "--reject-persist-rr is ignored\n");
#endif
    // an experiment on the filter's heading gain (exp/v9-observ): scaled by how well the moment observes the heading
    // (RawGnssEKFParams::hdg_obs_mode; the firmware's unless given: the mosaic's flight safeguards, else off)
#if RAW_REPLAY_HDGOBS
    if (a.hdg_obs_mode >= 0)
        cfg.ekf.hdg_obs_mode = static_cast<uint8_t>(a.hdg_obs_mode);
    if (a.hdg_obs_fh >= 0.0)
        cfg.ekf.hdg_obs_fh_mps2 = a.hdg_obs_fh;
    if (a.hdg_obs_tau >= 0.0)
        cfg.ekf.hdg_obs_tau_s = a.hdg_obs_tau;
    if (a.hdg_obs_sigma_v >= 0.0)
        cfg.ekf.hdg_obs_sigma_v = a.hdg_obs_sigma_v;
    if (a.hdg_obs_window >= 0.0)
        cfg.ekf.hdg_obs_window_s = a.hdg_obs_window;
    if (a.hdg_obs_open_g >= 0.0)
        cfg.ekf.hdg_obs_open_g = a.hdg_obs_open_g;
    if (a.hdg_obs_open_hold >= 0.0)
        cfg.ekf.hdg_obs_open_hold_s = a.hdg_obs_open_hold;
    if (a.hdg_obs_floor >= 0.0)
        cfg.ekf.hdg_obs_floor = a.hdg_obs_floor;
    if (a.hdg_obs_scope >= 0)
        cfg.heading_obs_scope = static_cast<uint8_t>(a.hdg_obs_scope);
#else
    if (a.hdg_obs_mode >= 0 || a.hdg_obs_scope >= 0)
        std::fprintf(stderr, "WARNING: this build has no heading observability scale (RAW_REPLAY_HDGOBS=OFF): "
                             "--hdg-obs-* are ignored\n");
#endif
#else
    // the rig's r20-r26 code (aef947c5): no receiver profile, no carrier, no full gate or tilt floor
    if (a.carrier > 0 || a.gate_full || a.tilt_sigma_deg > 0.0)
        std::fprintf(stderr, "WARNING: this build is the old code: --carrier, --gate-full and --tilt-sigma-deg do "
                             "not exist in it and are ignored\n");
#endif
    cfg.primary = true;
    cfg.min_step_s = 0.01;
#if RAW_REPLAY_CAPGOV
    // the mosaic-only image takes the profile's (main.cpp startRawShadow); the M1 keeps its own 2500, which is
    // every profile's
    cfg.epoch_slice_us = mosaic ? rx.epoch_slice_us : 2500.0;
#else
    cfg.epoch_slice_us = 2500.0;
#endif
    cfg.external_build = true;
    cfg.heading_mode = static_cast<uint8_t>(a.heading_mode);
    // --fw-defaults: the four bench flags below only where given, the firmware's defaults otherwise -- what
    // startRawShadow() builds without them, so a default the Beetle flies is replayed, not this harness's
    auto take = [&](const char* opt) { return !a.fw_defaults || a.given.count(opt) != 0; };
    if (take("--pad-sigma-deg"))
        cfg.heading_pad_sigma_deg = a.pad_sigma_deg;
    if (take("--handoff-deg"))
        cfg.heading_handoff_deg = a.handoff_deg;
#if !RAW_REPLAY_OLD
    // act's gate in flight: 0 the heading's, 1 whole from launch (A), 2 the heading's until the apogee call and
    // whole from it (H) -- TR_BENCH_HEADING_GATE_FULL
    if (take("--gate-full"))
        cfg.heading_gate = static_cast<uint8_t>(a.gate_full);
    if (take("--tilt-sigma-deg"))
        cfg.heading_launch_tilt_sigma_deg = a.tilt_sigma_deg;
    if (a.boost_screen >= 0)
        cfg.boost_screen = static_cast<uint8_t>(a.boost_screen);
    if (a.boost_screen_g >= 0.0)
        cfg.boost_screen_g = a.boost_screen_g;   // 0: every epoch before the call
    if (a.pad_sigma_fallback_deg > 0.0)
        cfg.heading_pad_sigma_fallback_deg = a.pad_sigma_fallback_deg;
    if (a.mag_recent_s >= 0.0)
        cfg.heading_mag_recent_s = a.mag_recent_s;
    if (a.level_heading_free >= 0)
        cfg.ekf.level_heading_free = a.level_heading_free != 0;
#endif
    if (a.max_epoch_age_s > 0.0)
        cfg.max_epoch_age_s = a.max_epoch_age_s;
#if !RAW_REPLAY_OLD
    // the pad's zero-velocity updates off (the pad ZUPT holds a static log's velocity to millimetres a second
    // whatever the receiver gives; the levelling stays: --gnss-only-from-s stops both)
    if (a.pad_zupt >= 0)
        cfg.pad_zupt = a.pad_zupt != 0;
#else
    if (a.pad_zupt >= 0 || a.gnss_only_from_s < 1.0e8)
        std::fprintf(stderr, "WARNING: this build is the old code: --pad-zupt and --gnss-only-from-s are ignored\n");
#endif
#if RAW_REPLAY_FITGATE
    if (a.rr_fit_gate >= 0.0)
        cfg.ekf.rr_fit_gate = a.rr_fit_gate;
#else
    if (a.rr_fit_gate >= 0.0)
        std::fprintf(stderr, "WARNING: this build has no fit gate (RAW_REPLAY_FITGATE=OFF): --rr-fit-gate is ignored\n");
#endif
#if RAW_REPLAY_LATCH
    if (a.vert_release_mps >= 0.0)
        cfg.vert_release_mps = a.vert_release_mps;
#else
    if (a.vert_release_mps >= 0.0)
        std::fprintf(stderr, "WARNING: this build has no latch (RAW_REPLAY_LATCH=OFF): --vert-release-mps is ignored\n");
#endif
#if RAW_REPLAY_VCHECK
    if (a.vert_check_mps >= 0.0)
        cfg.vert_check_mps = a.vert_check_mps;
    if (a.boost_screen_window_s >= 0.0)
        cfg.boost_screen_window_s = a.boost_screen_window_s;
#else
    if (a.vert_check_mps >= 0.0 || a.boost_screen_window_s >= 0.0)
        std::fprintf(stderr, "WARNING: this build has no vertical check or boost window (RAW_REPLAY_VCHECK=OFF): "
                             "--vert-check-mps and --boost-screen-window-s are ignored\n");
#endif
#if RAW_REPLAY_DVCHECK
    // the Doppler change check: each given switch over the firmware's (the mosaic's flight safeguards, else off)
    if (a.dv_check >= 0)
        cfg.dv_check = static_cast<uint8_t>(a.dv_check);
    if (a.dv_sat_k >= 0.0)
        cfg.dv_sat_k = a.dv_sat_k;
    if (a.dv_epoch_k >= 0.0)
        cfg.dv_epoch_k = a.dv_epoch_k;
    if (a.dv_floor >= 0.0)
        cfg.dv_floor_mps = a.dv_floor;
    if (a.dv_per_g >= 0.0)
        cfg.dv_sigma_per_g = a.dv_per_g;
    if (a.dv_att_deg >= 0.0)
        cfg.dv_att_allow_deg = a.dv_att_deg;
    if (a.dv_scale >= 0.0)
        cfg.dv_scale_allow = a.dv_scale;
    if (a.dv_bias >= 0.0)
        cfg.dv_bias_allow_mps2 = a.dv_bias;
    if (a.dv_lever_m >= 0.0)
        cfg.dv_lever_allow_m = a.dv_lever_m;
    if (a.dv_hold_s >= 0.0)
        cfg.dv_hold_s = a.dv_hold_s;
    if (a.dv_epoch_hold_s >= 0.0)
        cfg.dv_epoch_hold_s = a.dv_epoch_hold_s;
    if (a.dv_window_s >= 0.0)
        cfg.dv_window_s = a.dv_window_s;
    if (a.dv_deweight >= 0)
        cfg.dv_deweight = a.dv_deweight != 0;
    if (a.dv_ground >= 0)
        cfg.dv_ground = a.dv_ground != 0;
    if (a.dv_noise_tau_s >= 0.0)
        cfg.dv_noise_tau_s = a.dv_noise_tau_s;
    if (a.dv_hold_all_frac >= 0.0)
        cfg.dv_hold_all_frac = a.dv_hold_all_frac;
    if (a.dv_latch_k >= 0.0)
        cfg.dv_latch_k = a.dv_latch_k;
#if RAW_REPLAY_DVHDG
    if (a.dv_hdg_unset_deg >= 0.0)
        cfg.dv_hdg_unset_deg = a.dv_hdg_unset_deg;
#else
    if (a.dv_hdg_unset_deg >= 0.0)
        std::fprintf(stderr, "WARNING: this build has no heading allowance for a launch with no magnetometer heading "
                             "(RAW_REPLAY_DVHDG=OFF): --dv-hdg-unset-deg is ignored\n");
#endif
#else
    if (a.dv_check >= 0)
        std::fprintf(stderr, "WARNING: this build has no Doppler change check (RAW_REPLAY_DVCHECK=OFF): --dv-* are "
                             "ignored\n");
#endif
#if RAW_REPLAY_MOSAIC
    if (mosaic)
    {
        // startRawShadow() under TR_RAW_NAV_MOSAIC (build_units D1a): a restored flight starts 100 m / 20 m/s /
        // 15 deg unsure with the snapshot's accelerometer bias, and the levelling counts only fused epochs
        cfg.restore_pos_sigma_m = 100.0;
        cfg.restore_vel_sigma_mps = 20.0;
        cfg.restore_att_sigma_deg = 15.0;
        cfg.restore_seed_bias = true;
        cfg.level_on_fused_epochs = true;
    }
    if (a.unaided)
    {
        // the acceptance twin (plan section 4): the GNSS rows' gain masked on the attitude and the gyro bias at the
        // update only; the carrier update is not masked, so its fusion is refused (Appendix B 2)
        if (cfg.ekf.cr_mode == kCarrierFuse)
        {
            std::fprintf(stderr, "--unaided does not mask the carrier update: not with --carrier 2\n");
            return 2;
        }
        cfg.ekf.att_gnss_off = true;
    }
#endif
    // the turning body's allowances (RawGnssShadow.h): the firmware's unless given (the mosaic's flight safeguards,
    // else off)
#if RAW_REPLAY_ROT
    if (a.rot_lever_axial_m >= 0.0)
        cfg.rot_lever_axial_m = a.rot_lever_axial_m;
    if (a.rot_lever_radial_m >= 0.0)
        cfg.rot_lever_radial_m = a.rot_lever_radial_m;
    if (a.rot_lever_min_rps >= 0.0)
        cfg.rot_lever_min_rps = a.rot_lever_min_rps;
    if (a.att_hold_rate_rps >= 0.0)
        cfg.att_hold_rate_rps = a.att_hold_rate_rps;
    if (a.att_hold_s >= 0.0)
        cfg.att_hold_s = a.att_hold_s;
    if (a.rot_skip_rate_rps >= 0.0)
        cfg.rot_skip_rate_rps = a.rot_skip_rate_rps;
    if (a.rot_skip_s >= 0.0)
        cfg.rot_skip_s = a.rot_skip_s;
    if (a.rot_after_apogee)
        cfg.rot_after_apogee = true;
#endif
    // the antenna's offset, known: after the safeguards and every flag above (startRawShadow's order)
#if RAW_REPLAY_ANTENNA
    if (a.ant_sigma_axial_m >= 0.0)
        cfg.antenna_sigma_axial_m = a.ant_sigma_axial_m;
    if (a.ant_sigma_radial_m >= 0.0)
        cfg.antenna_sigma_radial_m = a.ant_sigma_radial_m;
    if (a.ant_min_rps >= 0.0)
        cfg.antenna_min_rps = a.ant_min_rps;
    if (a.ant_att_mode >= 0)
        cfg.antenna_att_mode = static_cast<uint8_t>(a.ant_att_mode);
    if (a.ant_known)
    {
        AntennaOffset ao;
        ao.known = true;
        for (int i = 0; i < 3; ++i)
            ao.b_m[i] = a.ant_m[i];
        applyAntennaOffset(ao, cfg);
    }
#else
    if (a.ant_known || a.ant_sigma_axial_m >= 0.0 || a.ant_sigma_radial_m >= 0.0 || a.ant_min_rps >= 0.0 ||
        a.ant_att_mode >= 0)
        std::fprintf(stderr, "WARNING: this build has no antenna offset (RAW_REPLAY_ANTENNA=OFF): --antenna-* are "
                             "ignored\n");
#endif
#if !RAW_REPLAY_ROT
    if (a.rot_lever_axial_m >= 0.0 || a.rot_lever_radial_m >= 0.0 || a.rot_lever_min_rps >= 0.0
        || a.att_hold_rate_rps >= 0.0 || a.att_hold_s >= 0.0 || a.rot_skip_rate_rps >= 0.0 || a.rot_skip_s >= 0.0
        || a.rot_after_apogee)
        std::fprintf(stderr, "WARNING: this build has no turning-body allowances (RAW_REPLAY_ROT=OFF): "
                             "--rot-lever-axial-m, --rot-lever-radial-m, --rot-lever-min-rate, --att-hold-rate, "
                             "--att-hold-s, --rot-skip-rate, --rot-skip-s and --rot-after-apogee are ignored\n");
#endif
    // an experiment on the clock drift's walk (the profile's 100 m^2/s^3 unless given): a tighter clock, which the
    // carrier wind-up's correction is a prerequisite for
    if (a.clk_drift_psd >= 0.0)
        cfg.ekf.clk_drift_psd = a.clk_drift_psd;
    // the spinning body (RawGnssShadow.h): the firmware's unless given (off)
#if RAW_REPLAY_WINDUP
    if (a.windup_k >= 0.0)
        cfg.windup_k = a.windup_k;
    if (a.windup_geom >= 0)
        cfg.windup_geom = a.windup_geom != 0;
    if (a.windup_monitor >= 0)
        cfg.windup_monitor = a.windup_monitor != 0;
    if (a.windup_monitor_k > 0.0)
        cfg.windup_monitor_k = a.windup_monitor_k;
    if (a.att_exact >= 0)
        cfg.ekf.att_exact = a.att_exact != 0;
    if (a.dv_lever_mode >= 0)
        cfg.dv_lever_mode = static_cast<uint8_t>(a.dv_lever_mode);
    if (a.dv_lever_radial_m >= 0.0)
        cfg.dv_lever_radial_m = a.dv_lever_radial_m;
#else
    if (a.windup_k >= 0.0 || a.windup_geom >= 0 || a.windup_monitor >= 0 || a.att_exact >= 0 || a.dv_lever_mode >= 0
        || a.dv_lever_radial_m >= 0.0 || a.windup_monitor_k > 0.0)
        std::fprintf(stderr, "WARNING: this build has no spinning-body switches (RAW_REPLAY_WINDUP=OFF): --windup-*, "
                             "--att-exact and --dv-lever-* are ignored\n");
#endif
#if RAW_REPLAY_MOSAIC
    const bool mosaic_eph_cap = gnss_raw::EphemerisStore::kCapacity == 128;
    if (mosaic != mosaic_eph_cap)
        std::fprintf(stderr, "WARNING: this binary's ephemeris store holds %u (the %s build's); the %s build's holds "
                             "%u: use %s\n",
                     static_cast<unsigned>(gnss_raw::EphemerisStore::kCapacity), mosaic_eph_cap ? "mosaic" : "Beetle",
                     mosaic ? "mosaic" : "Beetle", mosaic ? 128u : 64u, mosaic ? "raw_replay_mosaic" : "raw_replay");
#endif

    std::unique_ptr<RawGnssShadow> shp(new RawGnssShadow(cfg));
    std::unique_ptr<RawGnssMeasStage> stp(new RawGnssMeasStage(RawGnssShadow::stageConfig(cfg)));
    RawGnssShadow& sh = *shp;
    RawGnssMeasStage& stage = *stp;
    if (a.fc_unit_us > 0.0)
    {
        g_fake_unit_us = a.fc_unit_us;
        sh.setClock(&fakeClock);
    }
    lc86::Lc86Parser parser;

#if RAW_REPLAY_MOSAIC
    // ---- the mosaic-G5's stream: the driver's and the GNSS task's work on each SBF block (mosaic_feed.h), the
    // epochs on the profile's fused grid
    rr::MosaicFeed::Config fcfg;
    fcfg.fuse_period_ms = static_cast<uint32_t>(std::llround(fusedEpochPeriod(rx) * 1000.0));
#if RAW_REPLAY_DOPAVG
    // the Dopplers in pairs, as main.cpp's startRawShadow sets the driver
    fcfg.pair = rx.rr_pair_avg;
    fcfg.pair_cfg.dt_ms = static_cast<uint32_t>(std::llround(rx.raw_epoch_period_s * 1000.0));
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        fcfg.pair_cfg.cn0_min[k] = rx.sig[k].cn0_min_rr;
    fcfg.pair_cfg.max_diff_mps = rx.rr_pair_max_diff_mps;
    // a sim's epochs (X) take the same rule here: the ones between the grid's read for their Dopplers only
    const bool sim_grid = a.sim_fuse_grid || rx.rr_pair_avg;
    std::unique_ptr<mosaic::MosaicDopplerSet> sim_prevp(new mosaic::MosaicDopplerSet());
    std::unique_ptr<rr::SimEpoch> sim_pairp(new rr::SimEpoch());
    std::unique_ptr<RawEpoch> sim_rawp(new RawEpoch());
    mosaic::MosaicDopplerPairStats sim_pair_st;
    uint32_t sim_off_grid = 0;
    // offline only (--rr-pair-sigma): the averaged Dopplers by epoch (time of week, ms) and satellite (sys << 8 | prn),
    // their sigma scaled once built; and where epochs were left with Dopplers alone (flight time, usable, averaged)
    std::map<int64_t, std::vector<uint16_t>> paired_sats;
    std::vector<std::array<double, 3>> pair_alone_at;
    auto notePaired = [&](const RawEpoch& ep, uint64_t mask, double t_flight, int usable) {
        if (a.rr_pair_sigma > 0.0 && mask != 0)
        {
            std::vector<uint16_t>& v = paired_sats[std::llround(ep.tow_s * 1000.0)];
            for (int i = 0; i < ep.n; ++i)
                if (mask & (uint64_t{1} << i))
                    v.push_back(static_cast<uint16_t>(ep.obs[i].sys << 8 | ep.obs[i].prn));
        }
        int n = 0;
        for (int i = 0; i < ep.n; ++i)
            n += (mask >> i) & 1u;
        if (n < usable)
            pair_alone_at.push_back({t_flight, static_cast<double>(usable), static_cast<double>(n)});
    };
    auto scalePaired = [&](BuiltEpoch& b) {
        if (!(a.rr_pair_sigma > 0.0))
            return;
        auto it = paired_sats.find(std::llround(b.tow_s * 1000.0));
        if (it == paired_sats.end())
            return;
        for (int i = 0; i < b.n; ++i)
            if (b.meas[i].has_rr &&
                std::find(it->second.begin(), it->second.end(),
                          static_cast<uint16_t>(b.meas[i].sys << 8 | b.meas[i].prn)) != it->second.end())
                b.meas[i].sigma_rr = static_cast<float>(b.meas[i].sigma_rr * a.rr_pair_sigma);
        paired_sats.erase(it);
    };
    auto usableRates = [&](const RawEpoch& ep) {
        int u = 0;
        for (int i = 0; i < ep.n; ++i)
            if (ep.obs[i].has_rate && !(ep.obs[i].sys < GNSS_SYS_COUNT &&
                                        ep.obs[i].cn0_dbhz < fcfg.pair_cfg.cn0_min[ep.obs[i].sys]))
                u++;
        return u;
    };
#endif
    std::unique_ptr<rr::MosaicFeed> feedp(new rr::MosaicFeed(fcfg));
    rr::MosaicFeed& feed = *feedp;
    std::unique_ptr<mosaic::SbfParser> sbfp(new mosaic::SbfParser());
    struct BlockSink : mosaic::SbfSink
    {
        std::vector<std::vector<uint8_t>> blocks;
        void onBlock(const uint8_t* b, size_t len, size_t) override { blocks.emplace_back(b, b + len); }
        void onLine(const char*, size_t) override {}
        void onPrompt(const char*, size_t) override {}
    } sbf_sink;
    int eph_rinex = 0, eph_nav = 0, eph_n = 0, rinex_stale = 0;
    // ---- the day's broadcast file(s): each record handed to the stage when a receiver would first have had it,
    // for the satellites the epoch carries (rinex_nav.h), the two latest of each
    std::vector<rinex::NavRecord> rnx;
    for (const std::string& path : a.rinex)
    {
        rinex::NavFile nf;
        std::string err;
        if (!rinex::readNavFile(path, nf, err))
        {
            std::fprintf(stderr, "--rinex %s: %s\n", path.c_str(), err.c_str());
            return 1;
        }
        std::fprintf(stderr, "RINEX %s: %zu records (GPS, Galileo I/NAV, BeiDou MEO/IGSO); left out %d other "
                             "systems, %d F/NAV, %d GEO, %d bad\n",
                     path.c_str(), nf.recs.size(), nf.skipped_system, nf.skipped_fnav, nf.skipped_geo, nf.bad);
        rnx.insert(rnx.end(), nf.recs.begin(), nf.recs.end());
    }
    std::stable_sort(rnx.begin(), rnx.end(), [](const rinex::NavRecord& x, const rinex::NavRecord& y) {
        return x.avail_gps_s < y.avail_gps_s;
    });
    std::map<int, std::vector<size_t>> rnx_sat;   // (sys << 8) | prn -> its records by availability
    std::map<int, size_t> rnx_next;               //   and how many of them have been looked at
    {
        std::set<std::tuple<int, int, int, double>> seen;   // the merged file repeats an issue per station
        for (size_t i = 0; i < rnx.size(); ++i)
        {
            const gps_eph_t& e = rnx[i].eph;
            if (seen.insert(std::make_tuple(e.sys, e.prn, e.iode, rnx[i].toe_gps_s)).second)
                rnx_sat[(e.sys << 8) | e.prn].push_back(i);
        }
    }
    auto rinexFeed = [&](const RawEpoch& ep, double gps_s) {
        for (int i = 0; i < ep.n; ++i)
        {
            const int key = (ep.obs[i].sys << 8) | ep.obs[i].prn;
            auto it = rnx_sat.find(key);
            if (it == rnx_sat.end())
                continue;
            const std::vector<size_t>& v = it->second;
            size_t& k = rnx_next[key];
            size_t k1 = k;
            while (k1 < v.size() && rnx[v[k1]].avail_gps_s <= gps_s)
                k1++;
            // the two latest a receiver has by now (the store keeps two a satellite)
            for (size_t j = (k1 > k + 2 ? k1 - 2 : k); j < k1; ++j)
            {
                const rinex::NavRecord& r = rnx[v[j]];
                if (std::fabs(r.toe_gps_s - gps_s) > RawGnssMeasStage::kEphMaxOffsetS)
                {
                    rinex_stale++;
                    continue;
                }
                if (stage.addEphemeris(r.eph))
                    eph_rinex++;
            }
            k = k1;
        }
    };
    // ---- the nav blocks extract.py put at the start (R ... preload): the log begins long after the receiver did,
    // and it sends an issue only when it changes. Each is filed by the rule a RINEX record is (rinex::availGpsS:
    // when a receiver first has that issue), so one the receiver decoded later in the log (a later Galileo batch, a
    // new GPS upload) reaches the stage no earlier than it could have; its own line, where it was logged, files it
    // there as well
    struct PreNav
    {
        gps_eph_t eph;
        double avail_gps_s;
    };
    std::vector<PreNav> nav_pre;
    size_t nav_pre_next = 0, nav_pre_sorted = 0;
    int eph_pre = 0, eph_pre_filed = 0;
    auto preFeed = [&](double gps_s) {
        if (nav_pre_sorted != nav_pre.size())
        {
            std::stable_sort(nav_pre.begin() + static_cast<long>(nav_pre_next), nav_pre.end(),
                             [](const PreNav& x, const PreNav& y) { return x.avail_gps_s < y.avail_gps_s; });
            nav_pre_sorted = nav_pre.size();
        }
        for (; nav_pre_next < nav_pre.size() && nav_pre[nav_pre_next].avail_gps_s <= gps_s; ++nav_pre_next)
            if (stage.addEphemeris(nav_pre[nav_pre_next].eph))
                eph_pre_filed++;
    };
    // ---- the driver's time tag (rawTimeTagOk, mosaic_feed.h TimeTagGate): the clock line valid, FINETIME, a 1PPS
    // edge paired within 120 s (each pairing moves the line: a K line). Before the log's first ReceiverTime and K
    // line those first ones stand for the earlier (--time-tag-from-boot: nothing before them); a log with no
    // ReceiverTime at all is untimed (--assume-finetime: timed)
    rr::TimeTagGate tag_gate;
    tag_gate.from_boot = a.time_tag_from_boot;
    tag_gate.assume_finetime = a.assume_finetime;
    for (const Event& e : in.ev)
    {
        mosaic::SbfHeader h;
        mosaic::ReceiverTime rt;
        if (e.kind == 'R' && e.bytes.size() >= 8 && e.bytes[0] == '$' && e.bytes[1] == '@' &&
            mosaic::sbfHeader(e.bytes.data(), e.bytes.size(), h) && h.number == mosaic::SBF_RECEIVER_TIME &&
            mosaic::decodeReceiverTime(e.bytes.data(), e.bytes.size(), rt))
        {
            tag_gate.first_rt = (rt.sync_level & 0x04) ? 1 : 0;
            break;
        }
    }
    const int first_rt = tag_gate.first_rt;
    int untimed_not_fine = 0, untimed_pps_stale = 0, untimed_no_line = 0;
    // and the driver's MosaicTimeTag (mosaic_feed.h): no PVT's bias to carry, the last edge off the line, the chain
    // of biases not confirmed by the 1PPS since a break; a log's first chain stands for one confirmed before it
    int untimed_no_bias = 0, untimed_edge_off = 0, untimed_off_chain = 0, untimed_held = 0;
    if (!a.time_tag_from_boot)
        feed.assumeConfirmedAtStart();
    // ---- the flight loop's view (RawNavAdapter) and its apogee quorum (flight_votes.h)
    std::unique_ptr<gnss_raw::ShadowLink> linkp(new gnss_raw::ShadowLink());
    std::unique_ptr<RawNavAdapter> adapterp(new RawNavAdapter());
    RawNavAdapter& nav_adapter = *adapterp;
    nav_adapter.attach(linkp.get());
    // the flight's state (launch, landing): the IMU ticks' flags, a log's own state machine -- or, for an events
    // file with none behind it (a sim: META state_machine none) or --flight-state quorum, the quorum's own
    // (enterInflight on its launch flag, LANDED by landing_transition), which needs the quorum
    const bool state_from_quorum =
        a.flight_state == "quorum" ||
        (a.flight_state.empty() && in.meta.count("state_machine") && in.meta["state_machine"] == "none");
    const bool use_votes = mosaic || a.quorum || state_from_quorum;
    rr::FlightVotes::Config vcfg;
    vcfg.state_from_quorum = state_from_quorum;
    vcfg.gate_ekf_valid = !mosaic;   // the mosaic build admits GNSS against the barometer only (ga.ekf_valid false)
    // the firmware's config::RAW_VOTE_GUARD_ACTS: the guard only logs on the mosaic image (it is judged and
    // reported either way); --vote-guard 1 makes it act
    vcfg.guard = a.vote_guard >= 0 ? a.vote_guard != 0 : false;
    vcfg.loop_period_us = static_cast<uint32_t>(std::llround(1.0e6 / std::max(1.0, a.loop_hz)));
    std::unique_ptr<rr::FlightVotes> votesp(new rr::FlightVotes(vcfg));
    rr::FlightVotes& votes = *votesp;
    // --truth-nav: the same quorum on a perfect filter (the sim's truth published as the filter's state, the
    // run's own barometer, fixes and IMU ticks), report only -- its call is the one the counters' latency alone
    // gives, which the filter's is measured against. It withholds nothing (no guard) but says when the guard
    // would have latched on the truth
    rr::TruthNav truth_nav;
    std::unique_ptr<gnss_raw::ShadowLink> tlinkp;
    std::unique_ptr<RawNavAdapter> tadapterp;
    std::unique_ptr<rr::FlightVotes> tvotesp;
    if (!a.truth_nav.empty())
    {
        std::string why;
        if (!truth_nav.load(a.truth_nav, &why))
        {
            std::fprintf(stderr, "--truth-nav: %s\n", why.c_str());
            return 2;
        }
        if (!use_votes)
        {
            std::fprintf(stderr, "--truth-nav needs the quorum (--quorum, a mosaic or a sim)\n");
            return 2;
        }
        rr::FlightVotes::Config tcfg = vcfg;
        tcfg.guard = false;
        tlinkp.reset(new gnss_raw::ShadowLink());
        tadapterp.reset(new RawNavAdapter());
        tadapterp->attach(tlinkp.get());
        tvotesp.reset(new rr::FlightVotes(tcfg));
        tvotesp->setInitialized(true);
    }
    int64_t next_pass_us = INT64_MIN;
    int quorum_baro = 0;   // barometer samples the quorum got (none without a ground pressure)
    rr::SimCounts sim_counts;   // a sim's epochs (X) through the stage
    rr::SimLite sim_lite;       //   and, with --build-lite, the elevations it keeps
    bool sim_iono = false;      //   and the broadcast ionosphere its receiver had on the pad, once
#endif

    // ---- the pad's ephemerides and ionosphere (the pad context)
    int eph_pad = 0;
    for (const auto& fr : in.peph)
    {
        if (fr.size() <= rtcm3::kFrameOverhead)
            continue;
        const uint8_t* payload = fr.data() + 3;
        const size_t len = fr.size() - rtcm3::kFrameOverhead;
        if (stage.onEphemeris(payload, len))
            eph_pad++;
        sh.onEphemeris(payload, len);
    }
    // PNAV: 0 the launch tick, 1-3 lla, 4-6 velocity, 7-10 quaternion, 11-13 accelerometer bias, 14-16 gyro
    // bias, 17 clock bias, 18 drift, 19-20 Galileo and BeiDou offsets, 21 the barometer's offset, 22
    // declination, 23 field, 24-31 the broadcast ionosphere, 32 its day, 33 the ionosphere's source, 34 flags
    if (have_pnav && (static_cast<int>(in.pnav[34]) & 0x08))
    {
        ShadowIono io{};
        for (int i = 0; i < 8; ++i)
            io.raw[i] = static_cast<int8_t>(in.pnav[24 + i]);
        io.gps_day = static_cast<uint16_t>(in.pnav[32]);
        stage.setIono(io);
        sh.setIono(io);
    }
    const int iono_source = have_pnav ? static_cast<int>(in.pnav[33]) : -1;
    std::fprintf(stderr, "pad context: %s; ionosphere %s; declination %s\n", have_pnav ? "yes" : "NO",
                 (have_pnav && (static_cast<int>(in.pnav[34]) & 0x08)) ? "broadcast restored" : "none",
                 (have_pnav && (static_cast<int>(in.pnav[34]) & 0x10)) ? "restored" : "none");
    if (iono_source == 2)
        std::fprintf(stderr, "WARNING: this flight's ionosphere was the pad survey's: not restored, the replay "
                             "flies without it\n");

    // ---- the start: the first pad record, turned by the magnetometer's error difference (a cold start has none:
    // its seed is made on the pad below, once the receiver has a fix)
    std::vector<double> s_cold(40, 0.0);
    s_cold[7] = 1.0;
    const double* S = cold ? s_cold.data() : in.s.data();
    const int64_t t0_us = cold ? INT64_MIN : static_cast<int64_t>(S[0]);
    const double dpsi = (a.mag_error_deg - a.recorded_mag_error_deg) * kD2R;
    const double qz[4] = {std::cos(0.5 * dpsi), 0.0, 0.0, std::sin(0.5 * dpsi)};   // about NED down
    double q0[4] = {S[7], S[8], S[9], S[10]}, qs[4];
    quatMul(qz, q0, qs);
    if (a.have_pad_quat)   // the caller's pad attitude, final (its magnetometer error included)
        for (int i = 0; i < 4; ++i)
            qs[i] = a.pad_quat[i];
    const double cz = std::cos(dpsi), sz = std::sin(dpsi);
    double v0[3] = {cz * S[4] - sz * S[5], sz * S[4] + cz * S[5], S[6]};

    ShadowSeed seed{};
    seed.time_us = static_cast<uint32_t>(t0_us);
    for (int i = 0; i < 3; ++i)
    {
        seed.lla[i] = S[1 + i];
        seed.vel_ned[i] = static_cast<float>(v0[i]);
        seed.gyro_bias[i] = static_cast<float>(S[14 + i] - a.pad_wb_sub[i]);
    }
    for (int i = 0; i < 4; ++i)
        seed.quat[i] = static_cast<float>(qs[i]);
    seed.on_pad = 1;
    seed.pad_valid = 0;   // the survey's reference: not used with the broadcast ionosphere
    if (have_pnav && (static_cast<int>(in.pnav[34]) & 0x10))
    {
        seed.declination_rad = static_cast<float>(in.pnav[22] + dpsi);
        seed.field_ut = static_cast<float>(in.pnav[23]);
        seed.have_declination = 1;
    }
    seed.start_gen = 1;

    // the covariance: the pad context's (the launch tick), its velocity block
    // scaled to the first record's sigmas, the NED blocks turned with the state
    float P0[19][19] = {};
    bool have_p0 = false;
    if (have_pcov)
    {
        int k = 0;
        for (int i = 0; i < 19; ++i)
            for (int j = i; j < 19; ++j)
            {
                P0[i][j] = P0[j][i] = static_cast<float>(in.pcov[k]);
                k++;
            }
        const double* sig = &S[23];   // pos n e d, vel n e d (the first record's)
        double d[19];
        for (int i = 0; i < 19; ++i)
            d[i] = 1.0;
        for (int i = 3; i < 6; ++i)
            if (sig[i] > 0.0 && P0[i][i] > 0.0f)
                d[i] = sig[i] / std::sqrt(static_cast<double>(P0[i][i]));
        for (int i = 0; i < 19; ++i)
            for (int j = 0; j < 19; ++j)
                P0[i][j] = static_cast<float>(P0[i][j] * d[i] * d[j]);
        have_p0 = true;
    }
    if (have_p0 && dpsi != 0.0)
    {
        // T = blockdiag(Rz, Rz, I...): position and velocity are NED, the attitude error is the body's
        double T[19][19] = {};
        for (int i = 0; i < 19; ++i)
            T[i][i] = 1.0;
        for (int b = 0; b < 2; ++b)
        {
            const int o = 3 * b;
            T[o][o] = cz;
            T[o][o + 1] = -sz;
            T[o + 1][o] = sz;
            T[o + 1][o + 1] = cz;
        }
        double TP[19][19] = {};
        for (int i = 0; i < 19; ++i)
            for (int j = 0; j < 19; ++j)
                for (int k = 0; k < 19; ++k)
                    TP[i][j] += T[i][k] * P0[k][j];
        for (int i = 0; i < 19; ++i)
            for (int j = 0; j < 19; ++j)
            {
                double s = 0.0;
                for (int k = 0; k < 19; ++k)
                    s += TP[i][k] * T[j][k];
                P0[i][j] = static_cast<float>(s);
            }
    }

    if (a.reanchor && dpsi != 0.0)
        std::fprintf(stderr, "WARNING: --reanchor takes the recorded launch state, which carries the recorded "
                             "magnetometer error: not done with a different one\n");

    // ---- the CSV
    FILE* out = std::fopen(a.out.c_str(), "w");
    if (!out)
    {
        std::fprintf(stderr, "cannot write %s\n", a.out.c_str());
        return 1;
    }
    std::fprintf(out,
                 "t_mcu_s,t_s,running,in_flight,qw,qx,qy,qz,lat_deg,lon_deg,h_m,n_m,e_m,d_m,vn,ve,vd,"
                 "sig_tilt_n_deg,sig_tilt_e_deg,sig_hdg_deg,sig_pn,sig_pe,sig_pd,sig_vn,sig_ve,sig_vd,"
                 "ab_x,ab_y,ab_z,wb_x_dps,wb_y_dps,wb_z_dps,clk_b_m,clk_d_mps,healthy,finite,gnss_aided,baro_aided,"
                 "epochs,used_pr,used_rr,rej_pr,rej_rr,last_meas,age_ms,eph_sats,tracked,cr_built,cr_in,cr_used,"
                 "cr_half,cr_other,levelled,zupts,baro_fused,baro_held,mag_fused,hdg_fit_psi_deg,hdg_fit_sig_deg,"
                 "hdg_handoffs,stale\n");
#if RAW_REPLAY_MOSAIC
    // the epochs file (RUN.csv.epochs.csv, a mosaic or cold-start replay): one row per epoch the filter closed,
    // whatever --every -- its measurements fused and refused, the longest any one satellite has been refused running
    // (each kind's: the EKF's count times that kind's period), and the corrupted satellites built into it. The CSV's
    // own epoch fields are the last epoch at each row it writes, and a row written --every N can miss one
    FILE* epf = nullptr;
    if (mosaic || cold)
    {
        epf = std::fopen((a.out + ".epochs.csv").c_str(), "w");
        if (epf)
            std::fprintf(epf, "t_s,t_close_s,tow_ms,used_pr,rej_pr,used_rr,rej_rr,nis_max,rej_run_pr_s,rej_run_rr_s,"
                              "corrupt_rr,corrupt_pr%s%s\n",
#if RAW_REPLAY_ANTENNA
                         // the lever roll (RawGnssShadow.h: the change check's epochs fitted for it so far, the
                         // last one's roll correction and 1-sigma, deg) and the Dopplers' turning-body allowance
                         ",lever_roll_epochs,lever_roll_deg,lever_roll_sigma_deg,rot_sigma_mps",
#else
                         "",
#endif
#if RAW_REPLAY_WINDUP
                         // the epoch's clock drift, screens and checks, and the spinning body's (the windup study)
                         ",clk_d_mps,screen_rr,screen_refused,vert_dv_down,dv_chi2,dv_c_mps,dv_wu_dc_mps,"
                         "dv_lever_mps,dv_held_rr,wu_p_rps,wu_l1_mps,wumon_p_rps,wumon_sig_rps,wumon_dp_rps,"
                         "wumon_dp_sig_rps,att_sig_n_deg,att_sig_e_deg,att_sig_d_deg"
#else
                         ""
#endif
            );
    }
    // --pvt-out: each receiver fix with a solution, its height above the ellipsoid (the pad height's score)
    FILE* pvtf = nullptr;
    if (!a.pvt_out.empty())
    {
        pvtf = std::fopen(a.pvt_out.c_str(), "w");
        if (pvtf)
            std::fprintf(pvtf, "t_mcu_s,t_s,h_ell_m,undulation_m,mode,v_acc_m\n");
    }
#endif
#if RAW_REPLAY_DIGEST
    FILE* dg = nullptr;
    uint32_t dg_step = 0;
    int64_t dg_pcx_t0 = INT64_MIN / 2;   // none taken yet: the first goes when it is due
    if (!a.digest.empty())
    {
        dg = std::fopen(a.digest.c_str(), "w");
        if (!dg)
        {
            std::fprintf(stderr, "cannot write %s\n", a.digest.c_str());
            return 1;
        }
    }
#else
    if (!a.digest.empty())
    {
        std::fprintf(stderr, "--digest needs a build with RAW_REPLAY_VCHECK and RAW_REPLAY_LATCH (and not "
                             "RAW_REPLAY_OLD)\n");
        return 2;
    }
#endif

    // the pad's NED (the truth's origin): --pad, else the events file's META pad_lla (deg, deg, m), else on a cold
    // start the filter's seed (set where it is made)
    if (!a.given.count("--pad") && in.meta.count("pad_lla"))
        std::sscanf(in.meta["pad_lla"].c_str(), "%lf %lf %lf", &a.pad_lat, &a.pad_lon, &a.pad_h);
    const bool pad_from_seed = cold && !a.given.count("--pad") && !in.meta.count("pad_lla");
#if !RAW_REPLAY_MOSAIC
    (void)pad_from_seed;   // a cold start is the mosaic path's
#endif
    const double A = 6378137.0, E2 = 0.0066943799901;
    double RN = 0.0, RE = 0.0;
    auto setPad = [&](double lat_deg, double lon_deg, double h) {
        a.pad_lat = lat_deg;
        a.pad_lon = lon_deg;
        a.pad_h = h;
        const double s0 = std::sin(a.pad_lat * kD2R);
        const double w0 = std::sqrt(1.0 - E2 * s0 * s0);
        RN = A * (1.0 - E2) / (w0 * w0 * w0) + a.pad_h;
        RE = A / w0 + a.pad_h;
    };
    setPad(a.pad_lat, a.pad_lon, a.pad_h);

    // the clock line: each K event from its time on; before the log's first one, that first one (the lines move
    // by a millisecond or so a minute) -- the driver's earlier lines never reached the log. Each K line is a 1PPS
    // edge paired (the mosaic's time tag needs one within 120 s): the first stands for the earlier ones too
    ClockLine line;
    for (const Event& e : in.ev)
        if (e.kind == 'K')
        {
#if RAW_REPLAY_MOSAIC
            if (!a.time_tag_from_boot)
                tag_gate.pair_us = e.t_us;
#endif
            line.valid = (e.flags & 1) != 0;
            line.gps0_s = e.v[0];
            line.mcu0_us = static_cast<int64_t>(e.v[1]);
            line.a_s = e.v[2];
            line.t_ref_s = e.v[3];
            line.drift = e.v[4];
            line.latency_s = e.v[5];
            break;
        }
    const bool log_has_line = line.valid;
    int fix_week = 0;
    uint32_t fix_tow_ms = 0;
    bool have_fix = false;
    struct PendingEpoch
    {
        int64_t release_us;
        TimedRawEpoch e;
    };
    std::vector<PendingEpoch> gnss_task;   // epochs the GNSS task has not built yet
    std::unique_ptr<BuiltEpoch> slot(new BuiltEpoch());
    std::unique_ptr<TimedRawEpoch> scratch(new TimedRawEpoch());
    BaroGatePolicy baro_gate;
    double p_ground = in.meta.count("ground_pressure_pa") ? std::atof(in.meta["ground_pressure_pa"].c_str()) : -1.0;
    if (cold && p_ground <= 0.0)
    {
        // a log with no flight (no snapshot): the pressure of its first second stands for the pad's
        double sum = 0.0;
        int n = 0;
        int64_t tb = INT64_MIN;
        for (const Event& e : in.ev)
            if (e.kind == 'B')
            {
                if (tb == INT64_MIN)
                    tb = e.t_us;
                if (e.t_us - tb > 1000000)
                    break;
                sum += e.v[0];
                n++;
            }
        if (n > 0)
        {
            p_ground = sum / n;
            std::fprintf(stderr, "no ground pressure in %s: the first second's mean (%d samples) stands for it\n",
                         a.in.c_str(), n);
        }
    }
    double bmp_p = 0.0;
    int64_t bmp_t = 0;
    bool bmp_new = false;
    double mag_v[3] = {};
    int64_t mag_t = 0;
    bool mag_new = false;
    bool started = false, restored = false;
    int64_t last_seed_us = 0;
    uint32_t last_step = 0;
    int rows = 0;
    int first_epochs_seeded = 0;
    double first_tow = -1.0;
    int epochs_decoded = 0, epochs_untimed = 0, eph_flight = 0, ages_matched = 0, epochs_line_fallback = 0;
    int epochs_cut = 0, epochs_corrupted_sats = 0;
    // the corrupted epoch: its time of week (ms), flight time, and each satellite's errors (before the stage's
    // cap: how many of them it built, and what the filter did with them, are the epochs file's)
    struct CorruptSat
    {
        uint8_t sys, prn;
        double d_rr, d_pr;
    };
    std::vector<CorruptSat> corrupt_sats;
    int64_t corrupt_tow_ms = -1;
    double corrupt_t_s = 0.0;
    int corrupt_built_rr = -1, corrupt_built_pr = -1;   // of them, built into the epoch with a range rate, a code
    int rates_seen = 0, rates_relock_dropped = 0;
    bool corrupted = false;
    bool have_line_ever = false;
    const int64_t t_inflight = in.meta.count("t_inflight_us") ? std::atoll(in.meta["t_inflight_us"].c_str()) : -1;
    bool reanchored = false;
    bool gate_switched = false;
    const int64_t t_apogee_flag = in.meta.count("t_apogee_flag_us") ? std::atoll(in.meta["t_apogee_flag_us"].c_str()) : -1;
    int64_t gate_switch_us = -1;
    if (a.gate_full_from_apogee)
        gate_switch_us = t_apogee_flag;
    else if (a.gate_full_at_s > -1.0e8)
        gate_switch_us = static_cast<int64_t>((a.gate_full_at_s - a.t_base_s) * 1.0e6);
    if ((a.gate_full_from_apogee || a.gate_full_at_s > -1.0e8) && (a.heading_mode != 2 || a.gate_full))
        std::fprintf(stderr, "WARNING: a gate switched in flight only means something in act with the heading-only "
                             "gate (--heading-mode 2 --gate-full 0)\n");
    double pitch_vote_t = -1e9, vel_vote_t = -1e9;
    int64_t act_at_us = -1;
    if (a.act_from_apogee)
        act_at_us = t_apogee_flag;
    else if (a.act_at_s > -1.0e8)
        act_at_us = static_cast<int64_t>((a.act_at_s - a.t_base_s) * 1.0e6);
    if (act_at_us > 0 && a.heading_mode != 1)
        std::fprintf(stderr, "WARNING: act opened late is meant to run in observe (--heading-mode 1)\n");
    bool act_opened = false, act_closed = false;
    float last_acc[3] = {};
    GuardStats gstats;
    std::vector<double> fw_dropped_tow;    // the epochs the firmware's boost screen dropped (time of week, s)
    std::vector<double> fw_dropped_t;      //   and their flight times (s)
    std::vector<double> vert_dropped_t;    // the epochs its vertical check dropped (flight time, s)
    // the Doppler change check: the epochs it failed, the epochs with satellites failing (and how many), the
    // epochs whose Dopplers it held (and how many), flight time
    std::vector<double> dv_epoch_fail_t, dv_sat_fail_t, dv_held_t;
    std::vector<int> dv_sat_fail_n, dv_held_n;
    std::vector<ScreenCmp> cmps;           // --compare-sequential
    uint8_t last_flags = 0;
    int nonfinite_rows = 0, unhealthy_rows = 0;
#if RAW_REPLAY_MOSAIC
    rr::GnssOnlyArm gnss_only(a.gnss_only_from_s,
                              a.gnss_only_gate == 1 ? rr::GnssOnlyArm::kGateLaunch : rr::GnssOnlyArm::kGateGround,
                              a.gnss_only_keep_level);
    int mag_handed = 0;                    // magnetometer samples handed to the filter, and their sizes (uT)
    std::vector<double> mag_norms;
    uint32_t dop_applied = 0;              // the epochs the filter closed: Dopplers fused, refused, largest NIS
    std::vector<int> dop_used;
    std::vector<float> dop_nis;
    int dop_rejected = 0, dop_rejected_epochs = 0;
    // the filter's stops (RawGnssShadow::stop: kStopGap, kStopNotFinite, kStopFar ...) as the rows saw them: a stop
    // on the pad restarts from the seed within the step, so the running column never shows it
    uint32_t stops_seen = 0;
    std::vector<std::pair<double, int>> stop_at;
#endif
#if RAW_REPLAY_HDGOBS
    // hdg_obs_mode's share of the heading each closed epoch took (flight time, s), with the mode on
    std::vector<std::pair<double, float>> hdg_obs_at;
#endif

    auto writeRow = [&](int64_t t_us) {
        const RawGnssEKF<float>& e = sh.filter();
        const RawGnssShadow::Stats& st = sh.stats();
#if RAW_REPLAY_MOSAIC
        if (st.stops != stops_seen)
        {
            stops_seen = st.stops;
            stop_at.emplace_back(t_us * 1e-6 + a.t_base_s, static_cast<int>(st.last_stop));
        }
#endif
        const double* lla = e.lla();
        const float* v = e.velNed();
        const float* q = e.quat();
        float att[3];
        e.attSigmaNed(att);
        bool finite = true;
        for (int i = 0; i < 3; ++i)
            finite = finite && std::isfinite(lla[i]) && std::isfinite(v[i]);
        for (int i = 0; i < 4; ++i)
            finite = finite && std::isfinite(q[i]);
        ShadowNav nav;
        sh.navState(nav);
        const bool healthy = nav.running && finite;
        if (!finite)
            nonfinite_rows++;
        if (!healthy)
            unhealthy_rows++;
        const double n_m = (lla[0] * kR2D - a.pad_lat) * kD2R * RN;
        const double e_m = (lla[1] * kR2D - a.pad_lon) * kD2R * RE * std::cos(a.pad_lat * kD2R);
        const double d_m = -(lla[2] - a.pad_h);
        const float* ab = e.accelBias();
        const float* wb = e.gyroBias();
        std::fprintf(out,
                     "%.6f,%.6f,%d,%d,%.7f,%.7f,%.7f,%.7f,%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,"
                     "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,"
                     "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.5f,%d,%d,%d,%d,"
                     "%u,%d,%d,%d,%d,%d,%.1f,%d,%d,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%.3f,%.3f,%u,%u\n",
                     t_us * 1e-6, t_us * 1e-6 + a.t_base_s, sh.running() ? 1 : 0,
                     (last_flags & kShadowInFlight) ? 1 : 0, q[0], q[1], q[2], q[3], lla[0] * kR2D, lla[1] * kR2D,
                     lla[2], n_m, e_m, d_m, v[0], v[1], v[2], att[0] * kR2D, att[1] * kR2D, att[2] * kR2D,
                     e.sigma(0), e.sigma(1), e.sigma(2), e.sigma(3), e.sigma(4), e.sigma(5), ab[0], ab[1], ab[2],
                     wb[0] * kR2D, wb[1] * kR2D, wb[2] * kR2D, e.clockBias(), e.clockDrift(), healthy ? 1 : 0,
                     finite ? 1 : 0, nav.gnss_aided, nav.baro_aided, st.epochs_applied, st.last.used_pr,
                     st.last.used_rr, st.last.rejected_pr, st.last.rejected_rr, st.last_meas, st.last_age_s * 1000.0,
                     sh.ephemerisSatellites(), st.tracked, CR(st.cr_built), CR(st.cr_in), CR(st.cr_used),
                     CR(st.cr_half), CR(st.cr_other), sh.levelled_ ? 1 : 0, st.zupts + st.pad_zupts, st.baro_fused, st.baro_held,
                     st.mag_fused, st.heading_last.psi * kR2D, st.heading_last.sigma * kR2D, st.heading_handoffs,
                     st.epochs_stale);
        rows++;
    };

    // A cold start's seed (the mosaic-only build has no other filter to start from): made on the pad once the
    // receiver's position is known -- META seed_lla (deg, deg, m above the geoid: a sim's), else its first fix --
    // through RawNavAdapter as main.cpp's start makes it: the position and the gyro's recent mean (init), the
    // accelerometer's tilt with the pad heading guess (quatFromAccelHeading), the WMM's declination and field at
    // the pad (TR_GeoMag; META declination_rad / field_ned_ut give them instead), then startSeed.
    bool seed_ready = !cold;
#if RAW_REPLAY_MOSAIC
    bool cold_pos_known = false;
    bool cold_pos_meta = false;   // from META seed_lla (a sim's): the start waits for 16 ticks of gyro, as it has them
    double cold_lla[3] = {};   // rad, rad, m above the geoid
    float cold_sep = 0.0f;     // the geoid above the ellipsoid
    uint16_t cold_wnc = 0;
    uint32_t cold_tow_ms = 0;
    if (cold && in.meta.count("seed_lla"))
    {
        double la = 0.0, lo = 0.0, h = 0.0;
        if (std::sscanf(in.meta["seed_lla"].c_str(), "%lf %lf %lf", &la, &lo, &h) == 3)
        {
            cold_pos_known = true;
            cold_pos_meta = true;
            cold_lla[0] = la * kD2R;
            cold_lla[1] = lo * kD2R;
            cold_lla[2] = h;
        }
    }
    float gyro_ring[16][3] = {};
    int gyro_n = 0;
    double model_field_ut = have_pnav && (static_cast<int>(in.pnav[34]) & 0x10) ? in.pnav[23] : 0.0;
    auto coldSeed = [&](const ShadowImu& s, int64_t ev_t_now) {
        EkfIMUData imu;
        imu.time_us = s.time_us;
        imu.acc_x = s.acc[0];
        imu.acc_y = s.acc[1];
        imu.acc_z = s.acc[2];
        const int n = std::min(gyro_n, 16);
        double gm[3] = {};
        for (int k = 0; k < n; ++k)
            for (int i = 0; i < 3; ++i)
                gm[i] += gyro_ring[k][i];
        imu.gyro_x = n > 0 ? gm[0] / n * kR2D : 0.0;
        imu.gyro_y = n > 0 ? gm[1] / n * kR2D : 0.0;
        imu.gyro_z = n > 0 ? gm[2] / n * kR2D : 0.0;
        if (a.have_seed_gyro)
        {
            imu.gyro_x = a.seed_gyro_dps[0];
            imu.gyro_y = a.seed_gyro_dps[1];
            imu.gyro_z = a.seed_gyro_dps[2];
        }
        EkfGNSSDataLLA g;
        g.time_us = s.time_us;
        g.lat_rad = cold_lla[0];
        g.lon_rad = cold_lla[1];
        g.alt_m = cold_lla[2];
        nav_adapter.setGeoidSeparation(cold_sep);
        nav_adapter.init(imu, g, EkfMagData{});
        float q[4];
        double mq[4];
        if (in.meta.count("seed_quat") &&
            std::sscanf(in.meta["seed_quat"].c_str(), "%lf %lf %lf %lf", &mq[0], &mq[1], &mq[2], &mq[3]) == 4)
        {
            // a sim's truth at the seed (META seed_quat_truth 1): the pad heading a magnetometer would have settled
            // on over a long pad wait, so turned by its error as the declination is (TR_BENCH_HEADING_ERROR_DEG),
            // and by a heading guess's error where there is none (--seed-heading-error-deg)
            const bool sim_truth = in.meta.count("seed_quat_truth") && std::atoi(in.meta["seed_quat_truth"].c_str());
            const double turn = (sim_truth ? (a.mag_error_deg - a.recorded_mag_error_deg) * kD2R : 0.0) +
                                a.seed_heading_error_deg * kD2R;
            const double qt[4] = {std::cos(0.5 * turn), 0.0, 0.0, std::sin(0.5 * turn)};   // about NED down
            double qo[4];
            quatMul(qt, mq, qo);
            for (int i = 0; i < 4; ++i)
                q[i] = static_cast<float>(qo[i]);
            if (sim_truth)
                std::fprintf(stderr, "the cold start's attitude is the sim's truth (META seed_quat) turned %+.1f deg "
                                     "about down\n",
                             turn * kR2D);
            else
                std::fprintf(stderr, "WARNING: the cold start's attitude is META seed_quat (extract.py "
                                     "--seed-quat-from-log: the log's own flight filter's). Its heading is that "
                                     "filter's, so a heading scored against that filter's quaternion passes by "
                                     "construction where nothing else turns it (a static pad)\n");
        }
        else
        {
            quatFromAccelHeading(s.acc[0], s.acc[1], s.acc[2],
                                 static_cast<float>((a.seed_heading_deg + a.seed_heading_error_deg) * kD2R), q);
        }
        nav_adapter.setQuaternion(q[0], q[1], q[2], q[3]);
        // the model's date: META decimal_year, else the receiver's first fix's
        double year = TR_GeoMag::WMM_EPOCH;
        if (in.meta.count("decimal_year"))
            year = std::atof(in.meta["decimal_year"].c_str());
        else if (cold_wnc != 0)
        {
            const double days = cold_wnc * 7.0 + cold_tow_ms * 1.0e-3 / 86400.0;   // since 1980-01-06
            int y = 1980;
            double d = days + 5.0;   // since 1980-01-01
            for (;;)
            {
                const int len = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
                if (d < len)
                {
                    year = y + d / len;
                    break;
                }
                d -= len;
                y++;
            }
        }
        else
            std::fprintf(stderr, "WARNING: no date for the magnetic model (META decimal_year): its epoch, %.1f\n",
                         year);
        double decl = 0.0;
        if (in.meta.count("declination_rad"))
            decl = std::atof(in.meta["declination_rad"].c_str());
        else
            decl = TR_GeoMag::declinationRad(cold_lla[0], cold_lla[1], cold_lla[2], year);
        // TR_BENCH_HEADING_ERROR_DEG: north turned, as the firmware's bench flag turns it
        decl += (a.mag_error_deg - a.recorded_mag_error_deg) * kD2R;
        nav_adapter.setDeclination(static_cast<float>(decl));
        float ned[3];
        double mn[3];
        if (in.meta.count("field_ned_ut") &&
            std::sscanf(in.meta["field_ned_ut"].c_str(), "%lf %lf %lf", &mn[0], &mn[1], &mn[2]) == 3)
        {
            for (int i = 0; i < 3; ++i)
                ned[i] = static_cast<float>(mn[i]);
        }
        else
        {
            TR_GeoMag::fieldNED_uT(cold_lla[0], cold_lla[1], cold_lla[2], year, ned);
        }
        nav_adapter.setMagReference(ned);
        model_field_ut = std::sqrt(static_cast<double>(ned[0]) * ned[0] + static_cast<double>(ned[1]) * ned[1] +
                                   static_cast<double>(ned[2]) * ned[2]);
        nav_adapter.startSeed(seed);
        seed.on_pad = 1;
        seed.pad_valid = 0;
        if (pad_from_seed)
            setPad(cold_lla[0] * kR2D, cold_lla[1] * kR2D, cold_lla[2] + cold_sep);
        const double sinp = 2.0 * (static_cast<double>(q[0]) * q[2] - static_cast<double>(q[3]) * q[1]);
        std::fprintf(stderr, "cold start at T%+.3f s: pitch %.2f deg, heading guess %.1f deg, declination %s, "
                             "field %.1f uT, gyro bias %.3f/%.3f/%.3f deg/s (%s)\n",
                     ev_t_now * 1e-6 + a.t_base_s, std::asin(std::max(-1.0, std::min(1.0, sinp))) * kR2D,
                     a.seed_heading_deg, in.meta.count("declination_rad") ? "META's" : "WMM2025's",
                     std::sqrt(ned[0] * ned[0] + ned[1] * ned[1] + ned[2] * ned[2]), imu.gyro_x, imu.gyro_y,
                     imu.gyro_z, a.have_seed_gyro ? "--seed-gyro-dps" : "the last 16 ticks' mean");
    };
#endif

    // The synthesized faults' clock. The rig's restored LC86G runs (and any run given --truth-t0-tow): each epoch's
    // GPS time of week less the scenario's T+0. A mosaic or cold-start replay has no scenario behind it: its MCU
    // time (the arrival where untimed) plus t_base_s, the flight time its CSV and quorum report in. The corrupted
    // epoch is the one nearest --corrupt-at-s: within half the epoch period (the LC86G's 1 s, else the fused grid's).
    const bool faults_on_flight_time = (mosaic || cold) && !a.given.count("--truth-t0-tow");
    double corrupt_tol_s = 0.5;
#if RAW_REPLAY_MOSAIC
    if (faults_on_flight_time)
        corrupt_tol_s = 0.5 * fusedEpochPeriod(rx) + 1.0e-6;
#endif

    // An epoch decoded and placed on the MCU clock (te.mcu_us; 0 untimed), as it leaves the receiver's driver:
    // the synthesized faults (--cut-*, --corrupt-*), the relock rule, then on to the GNSS task. The LC86G's
    // restored start (lc86g_restored) also seeds the pad's lock history and hands the epoch over at its
    // logged on-board age.
    auto handOver = [&](TimedRawEpoch& te, int64_t arr_us, bool lc86g_restored) {
        epochs_decoded++;
        if (te.mcu_us == 0)
            epochs_untimed++;
        {
            // its flight time
            const double t_gps = faults_on_flight_time
                                     ? static_cast<double>(te.mcu_us != 0 ? te.mcu_us : arr_us) * 1.0e-6 + a.t_base_s
                                     : te.epoch.tow_s - a.truth_t0_tow;
            if (t_gps >= a.cut_from_s && t_gps < a.cut_to_s)
            {
                epochs_cut++;
                return;   // the receiver gave nothing
            }
            // the corrupted epoch: the nearest to --corrupt-at-s, or with --corrupt-pr-epoch the first at or after
            // it on a whole GPS second (the pseudoranges are fused on that epoch, pr_period_s 1 s)
            const bool whole_s =
                std::fabs(te.epoch.tow_s - std::floor(te.epoch.tow_s + 0.5)) < 1.0e-3;
            const bool hit = a.corrupt_pr_epoch ? (t_gps >= a.corrupt_at_s - corrupt_tol_s && whole_s)
                                                : std::fabs(t_gps - a.corrupt_at_s) < corrupt_tol_s;
            if (!corrupted && hit)
            {
                corrupted = true;
                corrupt_tow_ms = static_cast<int64_t>(std::llround(std::fmod(te.epoch.tow_s, 604800.0) * 1000.0));
                corrupt_t_s = t_gps;
                uint32_t st = 2463534242u + static_cast<uint32_t>(a.corrupt_seed) * 7919u;
                auto rnd = [&st]() {   // xorshift32: uniform in [0, 1)
                    st ^= st << 13;
                    st ^= st >> 17;
                    st ^= st << 5;
                    return (st & 0xFFFFFF) / 16777216.0;
                };
                int idx[RawEpoch::kMaxObs], nidx = 0;
                for (int i = 0; i < te.epoch.n; ++i)
                    if (te.epoch.obs[i].has_pr && te.epoch.obs[i].has_rate)
                        idx[nidx++] = i;
                for (int c = 0; c < 3 && nidx > 0; ++c)
                {
                    const int j = static_cast<int>(rnd() * nidx) % nidx;
                    RawObs& o = te.epoch.obs[idx[j]];
                    const double d_rr = (rnd() < 0.5 ? -1.0 : 1.0) *
                                        (a.corrupt_rr_mps[0] + (a.corrupt_rr_mps[1] - a.corrupt_rr_mps[0]) * rnd());
                    const double d_pr = (rnd() < 0.5 ? -1.0 : 1.0) *
                                        (a.corrupt_pr_m[0] + (a.corrupt_pr_m[1] - a.corrupt_pr_m[0]) * rnd());
                    o.rate_mps += d_rr;
                    o.pr_m += d_pr;
                    corrupt_sats.push_back({o.sys, o.prn, d_rr, d_pr});
                    idx[j] = idx[--nidx];
                    epochs_corrupted_sats++;
                }
            }
        }
        // the relock rule (--relock-min-lock-ms): until the apogee call, a young lock's range rate is
        // left out; its pseudorange and phase are kept
        for (int i = 0; i < te.epoch.n; ++i)
        {
            RawObs& o = te.epoch.obs[i];
            if (!o.has_rate)
                continue;
            rates_seen++;
            const bool after_call = t_apogee_flag > 0 && te.mcu_us != 0 && te.mcu_us >= t_apogee_flag;
            if (a.relock_min_lock_ms > 0.0 && !after_call &&
                static_cast<double>(o.lock_ms) < a.relock_min_lock_ms)
            {
                o.has_rate = false;
                rates_relock_dropped++;
            }
        }
        // the satellites the pad had: long locked, last updated an epoch ago
        if (first_tow < 0.0)
            first_tow = te.epoch.tow_s;
        if (lc86g_restored && started && te.epoch.tow_s - first_tow < 2.5 && first_epochs_seeded < 3)
        {
            first_epochs_seeded++;
            RawMeasBuilder& b = stage.builder_;
            RawGnssEKF<float>& f = sh.ekf_;
            for (int i = 0; i < te.epoch.n; ++i)
            {
                const RawObs& o = te.epoch.obs[i];
                if (o.sys >= 3 || o.prn >= 64 || b.seen_[o.sys][o.prn])
                    continue;
                b.seen_[o.sys][o.prn] = true;
                b.first_[o.sys][o.prn] = te.epoch.tow_s - 600.0;
                b.last_[o.sys][o.prn] = te.epoch.tow_s - 1.0;
                const double t_prev = te.mcu_us * 1e-6 - 0.85;
                f.last_upd_[o.sys][o.prn][0] = t_prev;
                f.last_upd_[o.sys][o.prn][1] = t_prev;
            }
        }
        // handed over as the flight computer did: when its filter closed this epoch (the logged
        // age) less the slices before the close, never before the frame plus a build
        int64_t release = arr_us + static_cast<int64_t>(a.epoch_delay_ms * 1000.0);
        const uint32_t tow_ms = static_cast<uint32_t>(std::llround(te.epoch.tow_s * 1000.0));
        auto it = in.age_ms.find(tow_ms);
        if (lc86g_restored && a.match_ages && te.mcu_us != 0 && it != in.age_ms.end())
        {
            release = te.mcu_us + static_cast<int64_t>((it->second - a.age_lead_ms) * 1000.0);
            release = std::max(release, arr_us + 5000);
            ages_matched++;
        }
        gnss_task.push_back({release, te});
    };

    // A clock line (a K event), from the time it took effect. On the mosaic each is also a 1PPS edge paired: the
    // edge against that line's place for its second, as the driver's pairPps hands MosaicTimeTag, with the line's
    // restarts (the record's count where extract.py wrote it, else its references moving)
    auto lineOfK = [](const Event& k) {
        ClockLine l;
        l.valid = (k.flags & 1) != 0;
        l.gps0_s = k.v[0];
        l.mcu0_us = static_cast<int64_t>(k.v[1]);
        l.a_s = k.v[2];
        l.t_ref_s = k.v[3];
        l.drift = k.v[4];
        l.latency_s = k.v[5];
        return l;
    };
#if RAW_REPLAY_MOSAIC
    std::vector<Event> k_pending;   // the mosaic's K lines waiting for the block that paired their edge
    int k_unpaired = 0;             //   taken with no such block seen (at the next K line, or the log's end)
    uint32_t k_restarts = 0;
    int k_lost = -1;                // clock records missing by their seq (-1: the K lines carry none)
    bool k_prev = false;
    double k_prev_gps0 = 0.0, k_prev_mcu0 = 0.0, k_prev_seq = 0.0;
#endif
    auto applyK = [&](const Event& k) {
        have_line_ever = true;
        line = lineOfK(k);
#if RAW_REPLAY_MOSAIC
        tag_gate.pair_us = k.t_us;
        if (mosaic)
        {
            if ((k.flags & 2) != 0)
            {
                // the mosaic-only image logs a record at every edge paired: its seq moves by 0 or 1 between them
                if (k_lost < 0)
                    k_lost = 0;
                if (k_prev && k.v[6] > k_prev_seq + 1.0)
                    k_lost += static_cast<int>(k.v[6] - k_prev_seq - 1.0);
                k_prev_seq = k.v[6];
                k_restarts = static_cast<uint32_t>(k.v[7]);
            }
            else if (k_prev && (k.v[0] != k_prev_gps0 || k.v[1] != k_prev_mcu0))
            {
                k_restarts++;
            }
            k_prev = true;
            k_prev_gps0 = k.v[0];
            k_prev_mcu0 = k.v[1];
            const int64_t gs = std::llround(line.gpsAt(k.t_us));
            int64_t at = 0;
            const bool on = line.mcuAt(static_cast<double>(gs), at);
            feed.notePair(gs, k.t_us, on, at, line.drift, k_restarts);
            feed.setLineSecondUs(static_cast<int64_t>(std::llround(1.0e6 * (1.0 + line.drift))));
        }
#endif
    };

    for (const Event& ev : in.ev)
    {
#if RAW_REPLAY_MOSAIC
        // the flight loop's passes up to this event (the apogee quorum): each reads what came before it
        if (use_votes && next_pass_us != INT64_MIN)
            for (; next_pass_us < ev.t_us; next_pass_us += vcfg.loop_period_us)
            {
                votes.pass(next_pass_us, nav_adapter);
                if (tvotesp && truth_nav.publish(next_pass_us, *tlinkp))
                    tvotesp->pass(next_pass_us, *tadapterp);
            }
#endif
        // the GNSS task: each epoch built once its build is done (the filter's
        // position at that time), handed to the filter's task
        if (!gnss_task.empty())
        {
            size_t k = 0;
            for (; k < gnss_task.size() && gnss_task[k].release_us <= ev.t_us; ++k)
            {
#if RAW_REPLAY_MOSAIC
                // the preloaded issues and the day's broadcast ephemerides a receiver would have had by now (the
                // latter for this epoch's satellites)
                if (gnss_task[k].e.gps_week != 0)
                {
                    const double gps_s = gnss_task[k].e.gps_week * 604800.0 + gnss_task[k].e.epoch.tow_s;
                    if (nav_pre_next < nav_pre.size())
                        preFeed(gps_s);
                    if (!rnx.empty())
                        rinexFeed(gnss_task[k].e.epoch, gps_s);
                }
#endif
                ShadowNav nav;
                sh.navState(nav);
                if (stage.offer(gnss_task[k].e, nav, *slot))
                {
#if RAW_REPLAY_MOSAIC
                    // the corrupted epoch as built: which of its corrupted satellites the stage's cap kept, with a
                    // range rate and with a code
                    if (corrupt_tow_ms >= 0 &&
                        std::llround(std::fmod(slot->tow_s, 604800.0) * 1000.0) == corrupt_tow_ms)
                    {
                        corrupt_built_rr = corrupt_built_pr = 0;
                        for (int i = 0; i < slot->n; ++i)
                            for (const CorruptSat& c : corrupt_sats)
                                if (slot->meas[i].sys == c.sys && slot->meas[i].prn == c.prn)
                                {
                                    corrupt_built_rr += slot->meas[i].has_rr ? 1 : 0;
                                    corrupt_built_pr += slot->meas[i].has_pr ? 1 : 0;
                                }
                    }
#endif
#if RAW_REPLAY_MOSAIC && RAW_REPLAY_DOPAVG
                    scalePaired(*slot);
#endif
                    sh.onBuilt(*slot);
                }
                sh.onMeasStats(stage.stats());
            }
            gnss_task.erase(gnss_task.begin(), gnss_task.begin() + static_cast<long>(k));
        }

        switch (ev.kind)
        {
        case 'B':
            bmp_t = ev.t_us;
            bmp_p = ev.v[0];
            bmp_new = true;
#if RAW_REPLAY_MOSAIC
            // the loop's pressure altitude above the pad, each sample (bmp_new_for_kf)
            if (use_votes && p_ground > 0.0)
            {
                quorum_baro++;
                const float palt =
                    44330.0f * (1.0f - std::pow(static_cast<float>(bmp_p) / static_cast<float>(p_ground), 1.0f / 5.255f));
                votes.onBaro(ev.t_us, palt, static_cast<float>(bmp_p));
                if (tvotesp)
                    tvotesp->onBaro(ev.t_us, palt, static_cast<float>(bmp_p));
            }
#endif
            break;
#if RAW_REPLAY_MOSAIC
        case 'V':
        {
            // a receiver fix from the events file (a sim's PVT), as the flight loop gets each
            rr::FixIn f;
            f.t_us = ev.t_us;
            f.fix_mode = static_cast<uint8_t>(ev.v[0]);
            f.num_sats = static_cast<uint8_t>(ev.v[1]);
            f.alt_m = ev.v[2];
            f.vel_n = ev.v[3];
            f.vel_e = ev.v[4];
            f.vel_u = ev.v[5];
            if (use_votes)
                votes.onFix(f);
            if (tvotesp)
                tvotesp->onFix(f);
            break;
        }
        case 'X':
        {
            // a sim's epoch, released to the GNSS task now: the stage's turn for it (sim_epoch.h: seen, its lock
            // ages, the cap, the builder's floors, mask and noise), then the filter's task takes it as it takes
            // a built one. The receiver's GPSIon set, as the mosaic sends it on the pad, before the first
            if (ev.flags < 0 || ev.flags >= static_cast<int>(in.xep.size()))
                break;
            const rr::SimEpoch* xp = &in.xep[static_cast<size_t>(ev.flags)];
#if RAW_REPLAY_DOPAVG
            if (sim_grid)
            {
                // the driver's cut to the fused grid, and with the Dopplers in pairs its rule (MosaicRawEpoch.h): a
                // sim's cells carry no lock time or clock jump, so a pair is two consecutive epochs' Dopplers of one
                // satellite, both at their floor, within the bound
                const rr::SimEpoch& x0 = *xp;
                const int64_t tow_ms = std::llround(x0.tow_s * 1000.0);
                const int64_t rx_ms = static_cast<int64_t>(x0.week) * 604800000LL + tow_ms;
                RawEpoch& ep = *sim_rawp;
                ep.tow_s = x0.tow_s;
                ep.n = x0.n;
                for (int i = 0; i < x0.n; ++i)
                {
                    RawObs& o = ep.obs[i];
                    o = RawObs{};
                    o.sys = x0.meas[i].sys;
                    o.prn = x0.meas[i].prn;
                    o.has_pr = x0.meas[i].has_pr;
                    o.has_rate = x0.meas[i].has_rr;
                    o.cn0_dbhz = x0.meas[i].cn0_dbhz;
                    o.pr_m = x0.meas[i].pr;
                    o.rate_mps = x0.meas[i].rr;
                }
                if (tow_ms % static_cast<int64_t>(fcfg.fuse_period_ms) != 0)
                {
                    sim_off_grid++;
                    if (rx.rr_pair_avg)
                        mosaic::dopplerSetFromRawEpoch(ep, rx_ms, *sim_prevp);
                    break;   // the driver hands over none of these
                }
                if (rx.rr_pair_avg)
                {
                    uint64_t mask = 0;
                    const int usable = usableRates(ep);
                    const int np =
                        mosaic::averageDopplerPairs(ep, rx_ms, false, *sim_prevp, fcfg.pair_cfg, sim_pair_st, &mask);
                    sim_prevp->valid = false;
                    rr::SimEpoch& xq = *sim_pairp;
                    xq = x0;
                    xq.rr_paired = np > 0 ? 1 : 0;   // in pairs, or as it came (MosaicRawEpoch.h)
                    for (int i = 0; i < x0.n; ++i)
                    {
                        xq.meas[i].rr = ep.obs[i].rate_mps;
                        xq.meas[i].has_rr = ep.obs[i].has_rate;   // one left alone in an epoch in pairs: dropped
                    }
                    if (a.sim_pair_exact && truth_nav.size() > 1 && mask != 0)
                    {
                        // the diagnostic: the mean of the receiver's velocity at the pair's two instants less its
                        // velocity at their middle (the truth's, at the IMU), along each averaged satellite's line
                        // of sight, put back -- so the pair is the range rate at its middle exactly
                        auto vAt = [&](int64_t t_us, double v[3]) {
                            size_t lo = 0, hi = truth_nav.size() - 1;
                            if (t_us <= truth_nav.row(0).t_us) hi = 0;
                            while (hi - lo > 1)
                            {
                                const size_t mid = (lo + hi) / 2;
                                (truth_nav.row(mid).t_us <= t_us ? lo : hi) = mid;
                            }
                            const rr::TruthNav::Row& r0 = truth_nav.row(lo);
                            const rr::TruthNav::Row& r1 = truth_nav.row(hi);
                            const double w = r1.t_us > r0.t_us ? static_cast<double>(t_us - r0.t_us) /
                                                                     static_cast<double>(r1.t_us - r0.t_us) : 0.0;
                            const double ww = std::min(1.0, std::max(0.0, w));
                            for (int k = 0; k < 3; ++k)
                                v[k] = r0.v[k] + ww * (r1.v[k] - r0.v[k]);
                        };
                        const int64_t te_us = x0.mcu_us;
                        double va[3], vb[3], vm[3], dn[3];
                        vAt(te_us, va);
                        vAt(te_us - 50000, vb);
                        vAt(te_us - 25000, vm);
                        for (int k = 0; k < 3; ++k)
                            dn[k] = 0.5 * (va[k] + vb[k]) - vm[k];
                        ShadowNav nv;
                        sh.navState(nv);
                        double rx_e[3];
                        geo_to_ecef(nv.lla[0], nv.lla[1], nv.lla[2], rx_e);
                        const double sl = std::sin(nv.lla[0]), cl = std::cos(nv.lla[0]);
                        const double so = std::sin(nv.lla[1]), co = std::cos(nv.lla[1]);
                        const double de[3] = {dn[0] * -sl * co + dn[1] * -so + dn[2] * -cl * co,
                                              dn[0] * -sl * so + dn[1] * co + dn[2] * -cl * so,
                                              dn[0] * cl + dn[2] * -sl};
                        for (int i = 0; i < x0.n; ++i)
                        {
                            if (!(mask & (uint64_t{1} << i)))
                                continue;
                            double u[3], nu = 0.0;
                            for (int k = 0; k < 3; ++k)
                            {
                                u[k] = x0.meas[i].sat_pos[k] - rx_e[k];
                                nu += u[k] * u[k];
                            }
                            nu = std::sqrt(nu);
                            if (!(nu > 1.0e6))
                                continue;
                            xq.meas[i].rr += (u[0] * de[0] + u[1] * de[1] + u[2] * de[2]) / nu;
                        }
                    }
                    notePaired(ep, mask, ev.t_us * 1.0e-6 + a.t_base_s, usable);
                    xp = &xq;
                }
            }
#endif
            const rr::SimEpoch& x = *xp;
            if (!sim_iono)
            {
                sim_iono = true;
                ShadowIono io{};
                io.gps_day = static_cast<uint16_t>(x.week * 7 + static_cast<int>(x.tow_s / 86400.0));
                stage.setIono(io);
                sh.setIono(io);
            }
            ShadowNav nav;
            sh.navState(nav);
            if (rr::simOffer(stage, x, nav, *slot, sim_counts, &sim_lite))
            {
#if RAW_REPLAY_DOPAVG
                scalePaired(*slot);
#endif
                sh.onBuilt(*slot);
            }
            sh.onMeasStats(stage.stats());
            break;
        }
        case 'N':
            // an ephemeris from the events file, to the GNSS task's stage
            if (ev.flags >= 0 && ev.flags < static_cast<int>(in.neph.size()) && stage.addEphemeris(in.neph[ev.flags]))
                eph_n++;
            break;
#endif
        case 'M':
            mag_t = ev.t_us;
            for (int i = 0; i < 3; ++i)
                mag_v[i] = ev.v[i];
            mag_new = true;
            break;
        case 'F':
            fix_week = static_cast<int>(ev.v[0]);
            fix_tow_ms = static_cast<uint32_t>(ev.v[1]);
            have_fix = true;
            break;
        case 'K':
#if RAW_REPLAY_MOSAIC
            if (mosaic)
            {
                // held for the block that paired its edge (mosaic_feed.h); one still held never saw it: now
                for (const Event& k : k_pending)
                {
                    applyK(k);
                    k_unpaired++;
                }
                k_pending.clear();
                k_pending.push_back(ev);
                break;
            }
#endif
            applyK(ev);
            break;
        case 'R':
        {
#if RAW_REPLAY_MOSAIC
            if (ev.bytes.size() >= 2 && ev.bytes[0] == '$' && ev.bytes[1] == '@')
            {
                // the mosaic-G5's SBF: the driver's parser, then each whole block as the driver and the GNSS task
                // take it (mosaic_feed.h)
                sbf_sink.blocks.clear();
                sbfp->feed(ev.bytes.data(), ev.bytes.size(), sbf_sink);
                for (const std::vector<uint8_t>& blk : sbf_sink.blocks)
                {
                    // the block that pairs a held K line's edge: the driver's line moved here, before the block
                    int64_t pair_s = 0;
                    if (!k_pending.empty() && rr::MosaicFeed::pairingSecond(blk.data(), blk.size(), pair_s))
                    {
                        size_t used = 0;
                        for (; used < k_pending.size(); ++used)
                        {
                            const Event& k = k_pending[used];
                            if (std::llround(lineOfK(k).gpsAt(k.t_us)) > pair_s)
                                break;
                            applyK(k);
                        }
                        k_pending.erase(k_pending.begin(), k_pending.begin() + static_cast<std::ptrdiff_t>(used));
                    }
                    TimedRawEpoch& te = *scratch;
                    switch (feed.onBlock(blk.data(), blk.size(), ev.t_us, te))
                    {
                    case rr::MosaicFeed::Kind::EPOCH:
                    {
                        // its GPS time on the clock line (the PPS-fitted 0x9D), where the driver would tag it:
                        // FINETIME, a 1PPS edge paired within 120 s, the line valid, a PVT's bias to carry, the
                        // last edge on the line, the chain confirmed (rawMeasEpoch, MosaicTimeTag)
                        int64_t mcu = 0;
                        const bool placed = feed.anchored() && line.mcuAt(feed.gpsS(), mcu);
                        const rr::TimeTagGate::Why why =
                            tag_gate.judge(feed.haveTime(), feed.finetime(), line.valid, ev.t_us, feed.anchored(),
                                           feed.timeTag().edgeOnLine(), feed.timeTag().held(),
                                           feed.timeTag().onChain());
                        switch (why)
                        {
                        case rr::TimeTagGate::Why::NOT_FINETIME: untimed_not_fine++; break;
                        case rr::TimeTagGate::Why::PPS_STALE: untimed_pps_stale++; break;
                        case rr::TimeTagGate::Why::NO_LINE: untimed_no_line++; break;
                        case rr::TimeTagGate::Why::NO_BIAS: untimed_no_bias++; break;
                        case rr::TimeTagGate::Why::EDGE_OFF: untimed_edge_off++; break;
                        case rr::TimeTagGate::Why::OFF_CHAIN: untimed_off_chain++; break;
                        case rr::TimeTagGate::Why::HELD: untimed_held++; break;
                        case rr::TimeTagGate::Why::TIMED:
                            if (placed)
                                te.mcu_us = mcu + static_cast<int64_t>(a.epoch_shift_ms * 1000.0);
                            break;
                        }
                        // a whole second's tag, timed or held back, for its PVT's check against its edge
                        if (placed)
                            feed.noteWholeSecond(mcu, te.mcu_us != 0);
#if RAW_REPLAY_DOPAVG
                        if (fcfg.pair)
                            notePaired(te.epoch, feed.lastPaired(), ev.t_us * 1.0e-6 + a.t_base_s,
                                       usableRates(te.epoch));
#endif
                        handOver(te, ev.t_us, false);
                        break;
                    }
                    case rr::MosaicFeed::Kind::NAV:
                        if (ev.flags == 1)
                        {
                            // preloaded: filed when a receiver has it (preFeed)
                            nav_pre.push_back({feed.eph(), rinex::availGpsS(feed.eph())});
                            eph_pre++;
                        }
                        else if (stage.addEphemeris(feed.eph()))
                        {
                            eph_nav++;
                        }
                        eph_flight++;
                        break;
                    case rr::MosaicFeed::Kind::ION:
                        // the GNSS task's one rule chose it (mosaic_feed.h): its stage builds with it, and the
                        // filter's task takes the chosen set (raw_iono_chosen)
                        stage.setIono(feed.iono());
                        sh.setIono(feed.iono());
                        break;
                    case rr::MosaicFeed::Kind::PVT:
                    {
                        const mosaic::PvtGeodetic& p = feed.pvt();
                        if (p.has_solution && p.undulation_m > -1.0e10f)
                            nav_adapter.setGeoidSeparation(p.undulation_m);
                        // the fix the flight loop gets (SensorConverter::convertGNSSData's units)
                        const GNSSData& g = feed.gnss();
                        rr::FixIn f;
                        f.t_us = ev.t_us;
                        f.fix_mode = g.fix_mode;
                        f.num_sats = g.num_sats;
                        f.alt_m = g.alt_mm * 1e-3;
                        f.vel_n = g.vel_n_mmps * 1e-3;
                        f.vel_e = g.vel_e_mmps * 1e-3;
                        f.vel_u = g.vel_u_mmps * 1e-3;
                        if (use_votes)
                            votes.onFix(f);
                        if (tvotesp)
                            tvotesp->onFix(f);
                        if (pvtf && p.has_solution)
                            std::fprintf(pvtf, "%.6f,%.6f,%.4f,%.4f,%u,%.2f\n", ev.t_us * 1e-6,
                                         ev.t_us * 1e-6 + a.t_base_s, p.height_m,
                                         static_cast<double>(p.undulation_m), static_cast<unsigned>(p.mode),
                                         p.v_acc_cm == 65535 ? -1.0 : p.v_acc_cm * 0.01);
                        if (cold && !cold_pos_known && p.has_solution)
                        {
                            // the cold start's position: the receiver's first fix (above the geoid, and its
                            // separation), as the flight computer's start takes it
                            cold_pos_known = true;
                            cold_lla[0] = p.lat_rad;
                            cold_lla[1] = p.lon_rad;
                            cold_lla[2] = g.alt_mm * 1e-3;
                            cold_sep = p.undulation_m > -1.0e10f ? p.undulation_m : 0.0f;
                            cold_wnc = p.wnc;
                            cold_tow_ms = p.tow_ms;
                        }
                        break;
                    }
                    default:
                        break;
                    }
                }
                break;
            }
#endif
            for (size_t i = 0; i < ev.bytes.size(); ++i)
            {
                const lc86::Event pe = parser.feed(ev.bytes[i]);
                if (pe == lc86::Event::RTCM)
                {
                    const size_t n = parser.rtcmFrameLen();
                    if (n > rtcm3::kFrameOverhead)
                    {
                        const uint8_t* payload = parser.rtcmFrame() + 3;
                        const size_t len = n - rtcm3::kFrameOverhead;
                        const uint16_t mt = rtcm3::messageType(payload, len);
                        if ((mt == 1019 || mt == 1046 || mt == 1042) && len <= 72)
                        {
                            stage.onEphemeris(payload, len);
                            sh.onEphemeris(payload, len);
                            eph_flight++;
                        }
                    }
                }
                else if (pe == lc86::Event::MSM_EPOCH)
                {
                    const rtcm3::MsmEpoch& me = parser.msmEpoch();
                    TimedRawEpoch& te = *scratch;
                    te = TimedRawEpoch();
                    if (!lc86::rawEpochFromMsm(me, te.epoch))
                        continue;
                    te.arrival_us = ev.t_us;
                    if (have_fix && fix_week != 0)
                    {
                        const double gps_s =
                            lc86::epochGpsSeconds(me.gps_tow_ms, static_cast<uint16_t>(fix_week), fix_tow_ms);
                        te.gps_week = static_cast<uint16_t>(std::floor(gps_s / 604800.0));
                        int64_t mcu = 0;
                        if (line.mcuAt(gps_s, mcu))
                            te.mcu_us = mcu;
                    }
                    if (te.mcu_us != 0)
                        te.mcu_us += static_cast<int64_t>(a.epoch_shift_ms * 1000.0);
                    if (te.mcu_us == 0 && !have_line_ever && !log_has_line && a.no_line_ms >= 0.0)
                    {
                        te.mcu_us = ev.t_us + static_cast<int64_t>(a.no_line_ms * 1000.0);
                        epochs_line_fallback++;
                    }
                    handOver(te, ev.t_us, !cold);
                }
            }
            break;
        }
        case 'I':
        {
            if (ev.t_us < t0_us)
                break;   // before the first record: nothing to start from
            ShadowImu s{};
            s.time_us = static_cast<uint32_t>(ev.t_us);
            for (int i = 0; i < 3; ++i)
            {
                s.acc[i] = static_cast<float>(ev.v[i] - a.acc_sub[i]);
                s.gyro[i] = static_cast<float>(ev.v[3 + i] - a.gyro_sub[i]);
            }
            s.flags = static_cast<uint8_t>(ev.flags);
#if !RAW_REPLAY_OLD
            // the flight computer's apogee call (pyro_apogee_detected: kShadowApogee), latched from the log's apogee
            // flag on -- it latches from kinematics.apogee_flag in the same pass. The log's ticks carry no such
            // bit; the firmware's H gate and boost screen read it
            if (a.apogee_flag && t_apogee_flag > 0 && ev.t_us >= t_apogee_flag)
                s.flags |= kShadowApogee;
#endif
#if RAW_REPLAY_MOSAIC
            if (use_votes)
            {
                // the tick's flags from the quorum, as rawShadowPushImu builds them (FlightVotes::shadowFlags): the
                // apogee call (pyro_apogee_detected) and the levelling in flight (use_ahrs_acc) from the quorum's own
                // call, landed from its kinematics -- not the events file's, which were the logged filter's or a
                // sim's truth-timed ones; in flight and LANDED too with the quorum's own state (--flight-state)
                s.flags = votes.shadowFlags(static_cast<uint8_t>(ev.flags));
                if (!a.apogee_flag)
                    s.flags = static_cast<uint8_t>(s.flags & ~kShadowApogee);
            }
#endif
#if RAW_REPLAY_MOSAIC
            // --gnss-only-from-s: from then on the ground the filter has the IMU, GNSS and the barometer, as in
            // flight. The pad's zero-velocity updates and its levelling hold a static log's velocity to millimetres a
            // second and its tilt to the accelerometer whatever the receiver gives (they would score the pad, not
            // GNSS), so both are off; and act's launch edge opens the attitude's gate, which the ground's cos^4
            // holds near shut on a payload standing nose up (gnss_only_arm.h; --gnss-only-gate ground keeps it)
            if (gnss_only.asked())
            {
                const bool was_on = gnss_only.on();
                const int launches0 = gnss_only.launches();
                s.flags = gnss_only.tick(sh, ev.t_us * 1e-6 + a.t_base_s, ev.t_us * 1e-6, s.flags);
                if (!was_on && gnss_only.on())
                    std::fprintf(stderr, "GNSS only from T%+.3f s: no pad ZUPT, %s, %s\n",
                                 ev.t_us * 1e-6 + a.t_base_s,
                                 gnss_only.keepLevel() ? "the levelling kept (the descent as flown)" : "no levelling",
                                 gnss_only.gate() == rr::GnssOnlyArm::kGateLaunch
                                     ? "act's launch edge on the attitude"
                                     : "the ground's attitude gate kept (--gnss-only-gate ground)");
                if (gnss_only.launches() != launches0)
                    std::fprintf(stderr, "GNSS only: the launch edge at T%+.3f s (start %u): %s, heading 1-sigma "
                                         "%.0f deg, tilt floor %.1f deg\n",
                                 ev.t_us * 1e-6 + a.t_base_s, sh.stats().starts,
                                 gnss_only.priorDeg() < 0.0 ? "not act: the gate left the config's"
                                 : sh.gate_full_                ? "the whole attitude's gate"
                                                                : "the heading's gate",
                                 gnss_only.priorDeg(), sh.cfg_.heading_launch_tilt_sigma_deg);
            }
#elif !RAW_REPLAY_OLD
            // --gnss-only-from-s: from then on the ground no pad ZUPT and no levelling (the pad's zero-velocity
            // updates and its levelling hold a static log's velocity to millimetres a second whatever the receiver
            // gives). This build keeps the ground's attitude gate: cos^4 pitch, near shut on a payload standing
            // nose up, so GNSS reaches the gyro bias but hardly the tilt (a RAW_REPLAY_MOSAIC build opens it)
            if (a.gnss_only_from_s < 1.0e8 && ev.t_us * 1e-6 + a.t_base_s >= a.gnss_only_from_s &&
                !(s.flags & kShadowInFlight))
            {
                if (sh.cfg_.pad_zupt)
                {
                    sh.cfg_.pad_zupt = false;
                    std::fprintf(stderr, "GNSS only from T%+.3f s: no pad ZUPT, no levelling (the ground's attitude "
                                         "gate kept)\n",
                                 ev.t_us * 1e-6 + a.t_base_s);
                }
                s.flags = static_cast<uint8_t>(s.flags & ~kShadowLevel);
            }
#endif
            last_flags = s.flags;
#if RAW_REPLAY_MOSAIC
            // the flight loop's side of the tick (rawShadowPushImu): the state it reads brought forward through
            // it, the start's gyro mean, a cold start's seed once the pad's position is known, the quorum's IMU
            nav_adapter.noteImu(s);
            for (int i = 0; i < 3; ++i)
                gyro_ring[gyro_n % 16][i] = s.gyro[i];
            gyro_n++;
            // main.cpp seeds the gyro bias from its 16-tick ring, which a boot has filled long before the
            // receiver's first fix; a log that begins after that fix (the 10/16-17 V9 logs, a few ms in) would
            // otherwise seed from one or two ticks, 0.1 deg/s off on the ISM6's quantized means, and no static
            // pad observes the bias about the vertical
            if (cold && !seed_ready && cold_pos_known && (gyro_n >= 16 || a.have_seed_gyro))
            {
                coldSeed(s, ev.t_us);
                seed_ready = true;
            }
            if (use_votes)
            {
                votes.setInitialized(seed_ready);
                votes.onImu(ev.t_us, s.acc, s.gyro, (ev.flags & kShadowInFlight) != 0);   // the tick's own edge
                if (tvotesp)
                    tvotesp->onImu(ev.t_us, s.acc, s.gyro, (ev.flags & kShadowInFlight) != 0);
                if (next_pass_us == INT64_MIN)
                    next_pass_us = votes.nextPass(ev.t_us);
            }
#endif
            // the magnetometer the loop handed the adapter this tick. The descent arm (--gnss-only-keep-level)
            // flies as in flight from its switch on, where the filter takes no magnetometer (RawGnssShadow takes it
            // on the ground only): none handed after it
#if RAW_REPLAY_MOSAIC
            if (mag_new && gnss_only.on() && gnss_only.keepLevel())
                mag_new = false;
#endif
            if (mag_new)
            {
                mag_new = false;
                ShadowMag m{};
                m.time_us = static_cast<uint32_t>(mag_t);
                m.flags = static_cast<uint8_t>(a.mag_flags);
                for (int i = 0; i < 3; ++i)
                    m.mag[i] = static_cast<float>(mag_v[i]);
                sh.onMag(m);
#if RAW_REPLAY_MOSAIC
                mag_handed++;
                mag_norms.push_back(std::sqrt(mag_v[0] * mag_v[0] + mag_v[1] * mag_v[1] + mag_v[2] * mag_v[2]));
#endif
            }
            // the seed, once a second (on the pad while not in flight)
            if (seed_ready && (!started || ev.t_us - last_seed_us >= 1000000))
            {
                last_seed_us = ev.t_us;
                seed.time_us = static_cast<uint32_t>(ev.t_us);
                seed.on_pad = (s.flags & (kShadowInFlight | kShadowLanded)) ? 0 : 1;
                sh.onSeed(seed);
            }
            // the launch tick: the pad context's own state, if asked -- the state that tick found, so set
            // before it steps (and before act opens the attitude at the launch edge)
            if (a.reanchor && dpsi == 0.0 && !reanchored && have_pnav && have_pcov && t_inflight > 0 &&
                ev.t_us >= t_inflight && sh.running())
            {
                reanchored = true;
                RawGnssEKF<float>& f = sh.ekf_;
                const double* N = in.pnav.data();
                for (int i = 0; i < 3; ++i)
                {
                    f.lla_[i] = N[1 + i];
                    f.v_[i] = static_cast<float>(N[4 + i]);
                    f.ab_[i] = static_cast<float>(N[11 + i]);
                    f.wb_[i] = static_cast<float>(N[14 + i]);
                }
                for (int i = 0; i < 4; ++i)
                    f.q_[i] = static_cast<float>(N[7 + i]);
                f.clk_b_ = N[17];
                f.clk_d_ = static_cast<float>(N[18]);
                f.isb_[0] = static_cast<float>(N[19]);
                f.isb_[1] = static_cast<float>(N[20]);
                int k = 0;
                for (int i = 0; i < 19; ++i)
                    for (int j = i; j < 19; ++j)
                    {
                        f.P_[i][j] = f.P_[j][i] = static_cast<float>(in.pcov[k]);
                        k++;
                    }
            }
            // the diagnosis: an epoch in the window, due at this step, is held out of it and worked unit by unit after
            bool diag = false, guarded = false;
            BuiltEpoch held;
            const bool guards_on = a.min_pr > 0 || a.dop_rms_mps > 0.0 || a.dv_max_mps > 0.0 || a.dv_sigma > 0.0 ||
                                   a.dv_vert_max > 0.0 || a.dv_vert_hold > 0.0 ||
                                   a.step_deg > 0.0 || a.step_sigma > 0.0 || a.gate_chi2 > 0.0 || a.majority;
            if (sh.running() && sh.npend_built_ > 0 && !sh.job_.active)
            {
                const BuiltEpoch& b = sh.pend_built_[sh.pb_head_];
                const double te_f = b.mcu_us * 1e-6 + a.t_base_s;
                const bool steps = ev.t_us * 1e-6 - sh.t_prev_ >= sh.cfg_.min_step_s - 1.0e-6;   // as onImu decides
                if (steps && te_f >= a.diag_from_s && te_f <= a.diag_to_s && b.mcu_us * 1e-6 <= ev.t_us * 1e-6)
                {
                    diag = true;
                    held = b;
                    sh.npend_built_ = 0;
                }
                else if (guards_on && steps && b.mcu_us * 1e-6 <= ev.t_us * 1e-6)
                {
                    guarded = true;
                    held = b;
                    sh.npend_built_ = 0;
                }
            }
            // --compare-sequential: an epoch due at this step worked first on a copy of the filter with the boost
            // screen off -- the harness's sequential majority check, its verdict from the same state the firmware's
            // pre-pass (on the filter itself, below) judges from
            std::unique_ptr<RawGnssShadow> seq;
            uint32_t cmp_tow_ms = 0;
            double cmp_tow = 0.0, cmp_te = 0.0;
#if !RAW_REPLAY_OLD
            if (a.compare_sequential && !diag && !guarded && sh.running() && sh.npend_built_ > 0 && !sh.job_.active)
            {
                const BuiltEpoch& b = sh.pend_built_[sh.pb_head_];
                const bool steps = ev.t_us * 1e-6 - sh.t_prev_ >= sh.cfg_.min_step_s - 1.0e-6;
                if (steps && b.mcu_us * 1e-6 - sh.cfg_.epoch_latency_s <= ev.t_us * 1e-6)
                {
                    cmp_tow = b.tow_s;
                    cmp_te = b.mcu_us * 1e-6 + a.t_base_s;
                    cmp_tow_ms = static_cast<uint32_t>(std::llround(std::fmod(b.tow_s, 604800.0) * 1000.0));
                    seq.reset(new RawGnssShadow(sh));
                    seq->cfg_.boost_screen = RawGnssShadow::kScreenOff;
                    const uint32_t applied0 = seq->stats().epochs_applied;
                    seq->onImu(s, ev.t_us);
                    if (seq->stats().epochs_applied != applied0 + 1)
                        seq.reset();   // no job closed here (an epoch dropped as stale, say): nothing to compare
                }
            }
            const uint32_t fw_dropped0 = sh.stats().boost_dropped;
            const uint32_t fw_closed0 = sh.stats().epochs_applied + sh.stats().boost_dropped;
#endif
#if RAW_REPLAY_VCHECK
            const uint32_t vert_dropped0 = sh.stats().vert_dropped;
            const uint32_t vert_checked0 = sh.stats().vert_checked;
            const uint32_t screened0 = sh.stats().boost_screened;
            uint32_t closed0 = sh.stats().epochs_applied + sh.stats().boost_dropped + sh.stats().vert_dropped;
#if RAW_REPLAY_LATCH
            closed0 += sh.stats().vert_latched;
#endif
#endif
#if RAW_REPLAY_DVCHECK
            const uint32_t dv_checks0 = sh.stats().dv_checks;
            const uint32_t dv_ef0 = sh.stats().dv_epoch_fails, dv_sf0 = sh.stats().dv_sat_fails;
            const uint32_t dv_held0 = sh.stats().dv_held_rr;
#endif
            sh.onImu(s, ev.t_us);
#if RAW_REPLAY_DVCHECK
            if (sh.stats().dv_checks != dv_checks0 || sh.stats().dv_held_rr != dv_held0)
            {
                const double te_dv = sh.job_.te + a.t_base_s;
                if (sh.stats().dv_epoch_fails != dv_ef0)
                    dv_epoch_fail_t.push_back(te_dv);
                if (sh.stats().dv_sat_fails != dv_sf0)
                {
                    dv_sat_fail_t.push_back(te_dv);
                    dv_sat_fail_n.push_back(static_cast<int>(sh.stats().dv_sat_fails - dv_sf0));
                }
                if (sh.stats().dv_held_rr != dv_held0)
                {
                    dv_held_t.push_back(te_dv);
                    dv_held_n.push_back(static_cast<int>(sh.stats().dv_held_rr - dv_held0));
                }
                if (std::getenv("RAW_REPLAY_DVLOG") && sh.stats().dv_checks != dv_checks0)
                    std::fprintf(stderr, "dv t=%.3f chi2=%.2f win=%.2f noise=%.3f along=%.3f epoch_fails+%u "
                                         "sat_fails+%u held+%u latch=%u level=%.2f\n", te_dv,
                                 static_cast<double>(sh.stats().dv_last_chi2),
                                 static_cast<double>(sh.stats().dv_last_chi2_win),
                                 static_cast<double>(sh.stats().dv_noise_mps),
                                 static_cast<double>(sh.stats().dv_last_along), sh.stats().dv_epoch_fails - dv_ef0,
                                 sh.stats().dv_sat_fails - dv_sf0, sh.stats().dv_held_rr - dv_held0,
                                 sh.dv_latch_ ? 1u : 0u, static_cast<double>(sh.stats().dv_last_level_chi2));
                if (std::getenv("RAW_REPLAY_DVLOG") && std::atoi(std::getenv("RAW_REPLAY_DVLOG")) > 1
                    && sh.stats().dv_checks != dv_checks0)
                    std::fprintf(stderr, "  win sum %+.3f %+.3f %+.3f sf %+.2f %+.2f %+.2f bias %.3f lever %.3f "
                                         "cov %.4f %.4f %.4f\n",
                                 sh.dv_SW_[0], sh.dv_SW_[1], sh.dv_SW_[2], sh.dv_SF_[0], sh.dv_SF_[1], sh.dv_SF_[2],
                                 sh.dv_SB_, sh.dv_SL_, std::sqrt(sh.dv_CW_[0][0]), std::sqrt(sh.dv_CW_[1][1]),
                                 std::sqrt(sh.dv_CW_[2][2]));
            }
#endif
#if RAW_REPLAY_MOSAIC
            {
                // the navigation state for the flight loop, as the raw task writes it each step and pass
                ShadowNav nv;
                sh.navState(nv);
                linkp->nav.write(nv);
                // each epoch the filter closed: its Dopplers fused and refused, its largest NIS
                if (sh.stats().epochs_applied != dop_applied)
                {
                    dop_applied = sh.stats().epochs_applied;
                    const RawGnssStats& l = sh.stats().last;
                    dop_used.push_back(l.used_rr);
                    dop_rejected += l.rejected_rr;
                    if (l.rejected_rr > 0)
                        dop_rejected_epochs++;
                    dop_nis.push_back(l.nis_max);
#if RAW_REPLAY_HDGOBS
                    if (cfg.ekf.hdg_obs_mode != 0 && hdg_obs_at.size() < 20000)
                        hdg_obs_at.emplace_back(sh.built_.mcu_us * 1e-6 + a.t_base_s, sh.filter().hdgObsScale());
#endif
                    if (epf)
                    {
                        // the longest any one satellite has been refused running, each kind's: the EKF's own count
                        // (rejects_, which reject_persist lets back in at) times that kind's period -- an epoch the
                        // stage left the satellite out of neither counts nor ends the run
                        const double te = sh.built_.mcu_us * 1e-6 + a.t_base_s;
                        const double period[2] = {sh.cfg_.pr_period_s > 0.0 ? sh.cfg_.pr_period_s
                                                                            : fusedEpochPeriod(rx),
                                                  fusedEpochPeriod(rx)};
                        int run_n[2] = {0, 0};
                        for (int sy = 0; sy < 3; ++sy)
                            for (int pn = 0; pn < 64; ++pn)
                                for (int kd = 0; kd < 2; ++kd)
                                    run_n[kd] = std::max(run_n[kd], static_cast<int>(sh.ekf_.rejects_[sy][pn][kd]));
                        const double run_s[2] = {run_n[0] * period[0], run_n[1] * period[1]};
                        const int64_t tow_ms = static_cast<int64_t>(sh.stats().last_tow_ms);
                        const bool is_corrupt = corrupt_tow_ms >= 0 && tow_ms == corrupt_tow_ms;
                        std::fprintf(epf, "%.6f,%.6f,%lld,%d,%d,%d,%d,%.3f,%.3f,%.3f,%d,%d", te,
                                     ev.t_us * 1e-6 + a.t_base_s, static_cast<long long>(tow_ms), l.used_pr,
                                     l.rejected_pr, l.used_rr, l.rejected_rr, static_cast<double>(l.nis_max), run_s[0],
                                     run_s[1], is_corrupt ? std::max(corrupt_built_rr, 0) : 0,
                                     is_corrupt ? std::max(corrupt_built_pr, 0) : 0);
#if RAW_REPLAY_ANTENNA
                        std::fprintf(epf, ",%u,%.4f,%.4f,%.4f", sh.stats().lever_roll_epochs,
                                     static_cast<double>(sh.stats().lever_roll_last_deg),
                                     static_cast<double>(sh.stats().lever_roll_last_sigma_deg),
                                     static_cast<double>(sh.stats().rot_last_sigma_mps));
#endif
#if RAW_REPLAY_WINDUP
                        {
                            // the epoch's clock drift, screens and checks, and the spinning body's: what each did at
                            // each roll rate (the windup study). The change check's per-epoch numbers are its last
                            // check's; its held count the change since the row before.
                            const RawGnssShadow::Stats& S = sh.stats();
                            float sg[3];
                            sh.filter().attSigmaNed(sg);
                            static uint32_t held0 = 0;
                            const uint32_t held = S.dv_held_rr - held0;
                            held0 = S.dv_held_rr;
                            std::fprintf(epf, ",%.4f,%d,%d,%.3f,%.2f,%.4f,%.4f,%.3f,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                                              "%.3f,%.3f,%.3f",
                                         static_cast<double>(sh.filter().clockDrift()), l.screen_rr, l.screen_refused,
                                         static_cast<double>(l.screen_dv_down_mps), static_cast<double>(S.dv_last_chi2),
                                         static_cast<double>(S.dv_last_c_mps), static_cast<double>(S.dv_last_wu_dc_mps),
                                         static_cast<double>(S.dv_last_lever_mps), held,
                                         sh.job_.wu_ok ? sh.job_.wu_p : std::nan(""),
                                         static_cast<double>(S.windup_last_l1_mps),
                                         static_cast<double>(S.wumon_last_p_rps),
                                         static_cast<double>(S.wumon_last_sig_rps), static_cast<double>(S.wumon_dp_rps),
                                         static_cast<double>(S.wumon_dp_sig_rps), sg[0] * 57.29577951308232,
                                         sg[1] * 57.29577951308232, sg[2] * 57.29577951308232);
                        }
#endif
                        std::fprintf(epf, "\n");
                    }
                }
            }
#endif
#if RAW_REPLAY_VCHECK
            if (sh.stats().vert_dropped != vert_dropped0)
                vert_dropped_t.push_back(sh.built_.mcu_us * 1e-6 + a.t_base_s);
            uint32_t closed1 = sh.stats().epochs_applied + sh.stats().boost_dropped + sh.stats().vert_dropped;
#if RAW_REPLAY_LATCH
            closed1 += sh.stats().vert_latched;
#endif
            if (std::getenv("RAW_REPLAY_VLOG") && std::atoi(std::getenv("RAW_REPLAY_VLOG")) >= 3 && closed1 != closed0)
            {
                const RawGnssStats& l = sh.stats().last;
                double g_mem = 0.0;
#if RAW_REPLAY_DYNMEM
                g_mem = sh.job_.g_mem;
#endif
                std::fprintf(stderr, "VEPOCH t %+.3f n %d %s | pr %d+%d rej | rr %d+%d rej | nis_max %.1f | g_peak %.1f "
                                     "g_mem %.1f\n",
                             sh.built_.mcu_us * 1e-6 + a.t_base_s, sh.built_.n,
                             l.screen_dropped ? "DROPPED" : "fused", l.used_pr, l.rejected_pr, l.used_rr,
                             l.rejected_rr, static_cast<double>(l.nis_max), sh.job_.g_peak, g_mem);
                int k = 0;
                for (int i = 0; i < sh.built_.n && k < RawEpoch::kMaxObs; ++i)
                {
                    const RawMeas& mi = sh.built_.meas[i];
                    if (mi.sys > GNSS_SYS_BDS || mi.prn >= 64 || !mi.has_rr)
                        continue;
                    // the row's geometry: H = (-u_ned, 1), u from the receiver to the satellite; p = the
                    // filter's velocity direction along u (a step in thrust along the velocity shows as p)
                    const double* hr = sh.ekf_.vfit_h_[k];
                    const double el = std::asin(std::max(-1.0, std::min(1.0, hr[2]))) * 57.29577951308232;
                    const double az = std::atan2(-hr[1], -hr[0]) * 57.29577951308232;
                    const double vn = static_cast<double>(sh.ekf_.v_[0]), ve = static_cast<double>(sh.ekf_.v_[1]),
                                 vd = static_cast<double>(sh.ekf_.v_[2]);
                    const double vm = std::sqrt(vn * vn + ve * ve + vd * vd);
                    const double pv = vm > 1e-3 ? -(vn * hr[0] + ve * hr[1] + vd * hr[2]) / vm : 0.0;
                    std::fprintf(stderr, "VEROW  sys %d prn %3d  y %+9.2f  s %6.2f  el %5.1f az %6.1f  p %+5.2f\n",
                                 mi.sys, mi.prn, sh.ekf_.vfit_y_[k], sh.ekf_.vfit_s_[k], el, az < 0 ? az + 360.0 : az,
                                 pv);
                    k++;
                }
            }
            // RAW_REPLAY_VLOG: each epoch the vertical check fitted, and each the boost screen judged
            if (std::getenv("RAW_REPLAY_VLOG") &&
                (sh.stats().vert_checked != vert_checked0 || sh.stats().boost_screened != screened0))
            {
                const RawGnssStats& r = sh.job_.rs;
                std::fprintf(stderr, "VLOG t %+.3f tow %.1f n %d | vert %s fit %d out %d dv_down %+.2f | screen %s rr %d "
                                     "refused %d | g_peak %.1f\n",
                             sh.built_.mcu_us * 1e-6 + a.t_base_s, sh.built_.tow_s, sh.built_.n,
                             sh.stats().vert_checked != vert_checked0 ? "yes" : "no", r.screen_fit_n, r.screen_fit_out,
                             static_cast<double>(r.screen_dv_down_mps),
                             sh.stats().boost_screened != screened0 ? "yes" : "no", r.screen_rr, r.screen_refused,
                             sh.job_.g_peak + 1.0);
                // RAW_REPLAY_VLOG=3: below, every closed epoch with its rows
                // RAW_REPLAY_VLOG=2: the fit's rows (the measurements with a rate, in order), each with its
                // innovation, its 1-sigma and whether the fit kept it
                if (std::atoi(std::getenv("RAW_REPLAY_VLOG")) >= 2 && sh.stats().vert_checked != vert_checked0)
                {
                    int k = 0;
                    for (int i = 0; i < sh.built_.n && k < RawEpoch::kMaxObs; ++i)
                    {
                        const RawMeas& mi = sh.built_.meas[i];
                        if (mi.sys > GNSS_SYS_BDS || mi.prn >= 64 || !mi.has_rr)
                            continue;
                        std::fprintf(stderr, "VROW   sys %d prn %3d  y %+9.2f  s %6.2f  %s\n", mi.sys, mi.prn,
                                     sh.ekf_.vfit_y_[k], sh.ekf_.vfit_s_[k], sh.ekf_.vfit_in_[k] ? "in" : "OUT");
                        k++;
                    }
                }
            }
#endif
#if !RAW_REPLAY_OLD
            if (sh.stats().boost_dropped != fw_dropped0)
            {
                fw_dropped_tow.push_back(sh.stats().last_tow_ms * 1e-3);
                fw_dropped_t.push_back(sh.built_.mcu_us * 1e-6 + a.t_base_s);
            }
            // both closed this epoch at this step (the filter applied or dropped it, the copy applied it)
            if (seq && sh.stats().epochs_applied + sh.stats().boost_dropped == fw_closed0 + 1 &&
                seq->stats().last_tow_ms == cmp_tow_ms && sh.stats().last_tow_ms == cmp_tow_ms &&
                seq->job_.g_peak + 1.0 > sh.cfg_.boost_screen_g)
            {
                ScreenCmp c;
                c.tow = cmp_tow;
                c.t_flight = cmp_te;
                c.g = seq->job_.g_peak + 1.0;
                c.apogee = (s.flags & kShadowApogee) != 0;
                c.fw_screened = sh.job_.screen;
                c.fw_rr = sh.stats().last.screen_rr;
                c.fw_refused = sh.stats().last.screen_refused;
                c.fw_drop = sh.stats().last.screen_dropped;
                c.seq_used = seq->stats().last.used_rr;
                c.seq_rej = seq->stats().last.rejected_rr;
                c.seq_drop = c.seq_rej > 0 && c.seq_rej >= c.seq_used;
                c.meas = seq->stats().last_meas;
                if (sh.stats().boost_last_dry_used >= 0 || sh.stats().last.screen_seq_used > 0 ||
                    sh.stats().last.screen_seq_refused > 0)
                {
                    c.fw_dry_used = sh.stats().last.screen_seq_used;
                    c.fw_dry_rej = sh.stats().last.screen_seq_refused;
                }
                c.seq_used_pr = seq->stats().last.used_pr;
                c.seq_rej_pr = seq->stats().last.rejected_pr;
                cmps.push_back(c);
            }
#endif
            if (diag)
                diagEpoch(sh, held, ev.t_us, s.flags, a, last_acc);
            if (guarded && sh.running())
                guardedEpoch(sh, held, ev.t_us, s.flags, a, gstats);
            for (int i = 0; i < 3; ++i)
                last_acc[i] = s.acc[i];
            if (!started && sh.running())
            {
                started = true;
            }
            if (!restored && !cold && started)
            {
                // the pad state the log's first record carried
                RawGnssEKF<float>& f = sh.ekf_;
                for (int i = 0; i < 3; ++i)
                {
                    f.lla_[i] = S[1 + i];
                    f.v_[i] = static_cast<float>(v0[i]);
                    f.ab_[i] = static_cast<float>(a.have_pad_ab ? a.pad_ab[i] : S[11 + i]);
                    f.wb_[i] = static_cast<float>(S[14 + i] - a.pad_wb_sub[i]);
                }
                for (int i = 0; i < 4; ++i)
                    f.q_[i] = static_cast<float>(qs[i]);
                f.clk_b_ = S[17];
                f.clk_d_ = static_cast<float>(S[18]);
                f.isb_[0] = static_cast<float>(S[19]);
                f.isb_[1] = static_cast<float>(S[20]);
                f.clk_ready_ = S[21] != 0.0;
                if (have_p0)
                    for (int i = 0; i < 19; ++i)
                        for (int j = 0; j < 19; ++j)
                            f.P_[i][j] = P0[i][j];
                sh.own_fix_ = true;
                if (have_pnav)
                {
                    sh.baro_offset_ = in.pnav[21];
                    sh.baro_learn_s_ = 60.0;
                    sh.baro_seen_ = (static_cast<int>(in.pnav[34]) & 0x04) != 0;
                }
                sh.last_aided_t_ = ev.t_us * 1e-6;
                restored = true;
            }
            // the gate switched in flight (harness only): act's full gate from then on, as the launch edge
            // sets it (RawGnssShadow::stepHeading), while the rocket is in flight
            if (!gate_switched && gate_switch_us > 0 && ev.t_us >= gate_switch_us && sh.running() &&
                (s.flags & kShadowInFlight))
            {
                gate_switched = true;
#if !RAW_REPLAY_OLD
                sh.ekf_.setAttGateVertical(false);
                sh.ekf_.setAttGateCos4(false);
#endif
            }
#if !RAW_REPLAY_OLD
            // act opened late (harness only): act's launch edge at that time, the gates back on the ground
            if (!act_opened && act_at_us > 0 && ev.t_us >= act_at_us && sh.running() && (s.flags & kShadowInFlight))
            {
                act_opened = true;
                if (a.gate_full == 1)
                {
                    sh.ekf_.setAttGateVertical(false);
                    sh.ekf_.setAttGateCos4(false);
                }
                else
                {
                    sh.ekf_.setAttGateVertical(true);
                }
                sh.ekf_.setHeadingSigma(static_cast<float>(a.pad_sigma_deg * kD2R));
                if (a.tilt_sigma_deg > 0.0)
                    sh.ekf_.setTiltSigmaFloor(static_cast<float>(a.tilt_sigma_deg * kD2R));
            }
            if (act_opened && !act_closed && !(s.flags & kShadowInFlight))
            {
                act_closed = true;
                sh.ekf_.setAttGateVertical(cfg.ekf.att_gate_vertical);
                sh.ekf_.setAttGateCos4(cfg.ekf.att_gate_cos4);
            }
#endif
            // the apogee votes the filter's state would cast (TR_KinematicChecks tests 1 and 4), at every step
            if (sh.running() && ev.t_us * 1e-6 + a.t_base_s > a.burnout_s)
            {
                const RawGnssEKF<float>& f = sh.filter();
                const float* q = f.quat();
                const double sinp = 2.0 * (static_cast<double>(q[0]) * q[2] - static_cast<double>(q[3]) * q[1]);
                const double pitch = std::asin(std::max(-1.0, std::min(1.0, sinp)));
                const double tf = ev.t_us * 1e-6 + a.t_base_s;
                if (pitch_vote_t < -1e8 && pitch < -0.087)
                    pitch_vote_t = tf;
                const double up = f.lla()[2] - a.pad_h;
                if (vel_vote_t < -1e8 && up > 15.0 && f.velNed()[2] > 0.0f)
                    vel_vote_t = tf;
            }
            // back to the GNSS task: the satellites the checks found jumped
            ShadowRelock rl;
            while (sh.takeRelock(rl))
                stage.markRelock(rl.sys, rl.prn, rl.tow_s);
            // the barometer sample the loop fused this tick (BaroGatePolicy)
            if (bmp_new && p_ground > 0.0)
            {
                bmp_new = false;
                const float alt = 44330.0f * (1.0f - std::pow(static_cast<float>(bmp_p) /
                                                               static_cast<float>(p_ground), 1.0f / 5.255f));
                bool baro_locked = false;
#if RAW_REPLAY_MOSAIC
                // the EKF tick's gate with the flight loop's: the transonic lockout and GnssAscentGate's stuck
                // barometer hold it out of the filter (main.cpp's barometer measurement update)
                if (use_votes)
                    baro_locked = votes.machLocked() || !votes.baroFusable(bmp_p >= 25000.0 && bmp_p <= 125000.0);
#endif
                const BaroGatePolicy::Decision d =
                    baro_gate.evaluate(static_cast<uint32_t>(bmp_t), alt, baro_locked, 5.0f, 500.0f, 20000u);
                if (d.accept)
                    sh.onBaro(ShadowBaro{static_cast<uint32_t>(bmp_t), alt});
            }
            // a row each step
            if (sh.running() && sh.stats().step_n != last_step)
            {
                if ((sh.stats().step_n % static_cast<uint32_t>(a.every)) == 0)
                    writeRow(ev.t_us);
                last_step = sh.stats().step_n;
            }
#if RAW_REPLAY_DIGEST
            // --digest: a line every step, whatever --every, and the step a stop happened at too
            if (dg && sh.stats().step_n != dg_step)
            {
                dg_step = sh.stats().step_n;
                writeDigest(dg, sh, s.time_us, ev.t_us, dg_pcx_t0);
            }
#endif
            break;
        }
        default:
            break;
        }
    }
#if RAW_REPLAY_MOSAIC
    k_unpaired += static_cast<int>(k_pending.size());   // the log ended before their blocks
#endif
    std::fclose(out);
#if RAW_REPLAY_MOSAIC
    if (epf)
        std::fclose(epf);
    if (pvtf)
        std::fclose(pvtf);
#endif
#if RAW_REPLAY_DIGEST
    if (dg)
    {
        // and where the replay ended: the counters that move between steps (epochs seen while stopped, say)
        ShadowNav nav;
        sh.navState(nav);
        std::fprintf(dg, "end %u %s %s %s\n", sh.stats().step_n, hex64(navHash(nav)).c_str(),
                     hex64(statsHash(sh.stats())).c_str(), hex64(stateHash(sh.filter())).c_str());
        std::fclose(dg);
    }
#endif
    {
        FILE* ej = std::fopen((a.out + ".events.json").c_str(), "w");
        if (ej)
        {
            std::fprintf(ej, "{\"guard\": {\"boost_epochs\": %d, \"blocked_pr\": %d, \"blocked_doppler\": %d, "
                             "\"refused\": %d, \"tightened\": %d, \"rejected\": %d, \"majority\": %d}, "
                             "\"cut\": %d, \"corrupted_sats\": %d, \"rates_seen\": %d, \"rates_relock_dropped\": %d, ",
                         gstats.epochs, gstats.blocked_pr, gstats.blocked_dop, gstats.refused, gstats.tightened,
                         gstats.rejected, gstats.majority, epochs_cut, epochs_corrupted_sats, rates_seen,
                         rates_relock_dropped);
            auto list = [&](const std::vector<double>& v) {
                std::string o = "[";
                for (size_t i = 0; i < v.size(); ++i)
                    o += (i ? ", " : "") + std::to_string(v[i]);
                return o + "]";
            };
            std::fprintf(ej, "\"majority_tow\": %s, ", list(gstats.majority_tow).c_str());
#if !RAW_REPLAY_OLD
            {
                const RawGnssShadow::Stats& fs = sh.stats();
                std::fprintf(ej, "\"fw\": {\"screened\": %u, \"dropped\": %u, \"dropped_tow\": %s, \"prior_deg\": %.3f, "
                                 "\"prior_mag\": %d, \"mag_age_s\": %.3f, \"act_launches\": %u, \"levels\": %u, "
                                 "\"levels_held\": %u, \"mag_fused\": %u, \"mag_uncal\": %u, \"gate_apogee\": %u, "
                                 "\"gate_apogee_s\": %.3f, \"dropped_dry\": %u}, ",
                             fs.boost_screened, fs.boost_dropped, list(fw_dropped_tow).c_str(),
                             static_cast<double>(fs.heading_launch_prior_deg), fs.heading_launch_mag,
                             static_cast<double>(fs.heading_launch_mag_age_s), fs.heading_act_launches, fs.levels,
                             fs.levels_held, fs.mag_fused, fs.mag_uncal, fs.heading_gate_apogee,
                             static_cast<double>(fs.heading_gate_apogee_s), fs.boost_dropped_dry);
#if RAW_REPLAY_VCHECK
                std::fprintf(ej, "\"vert\": {\"checked\": %u, \"dropped\": %u, \"dropped_t\": %s, "
                                 "\"last_dv_down\": %.2f, \"worst_kept_dv_down\": %.2f, \"threshold\": %.1f, "
                                 "\"window_s\": %.2f}, \"fw_dropped_t\": %s, ",
                             fs.vert_checked, fs.vert_dropped, list(vert_dropped_t).c_str(),
                             static_cast<double>(fs.vert_last_dv_down), static_cast<double>(fs.vert_worst_dv_down),
                             sh.cfg_.vert_check_mps, sh.cfg_.boost_screen_window_s, list(fw_dropped_t).c_str());
#endif
#if RAW_REPLAY_LATCH
                std::fprintf(ej, "\"vert_latched\": %u, ", fs.vert_latched);
#endif
#if RAW_REPLAY_DVCHECK
                {
                    auto ilist = [&](const std::vector<int>& v) {
                        std::string o = "[";
                        for (size_t i = 0; i < v.size(); ++i)
                            o += (i ? ", " : "") + std::to_string(v[i]);
                        return o + "]";
                    };
                    std::fprintf(ej, "\"dv\": {\"mode\": %d, \"checks\": %u, \"sat_fails\": %u, \"sat_win_fails\": %u, "
                                     "\"epoch_fails\": %u, \"epoch_win_fails\": %u, \"held_rr\": %u, "
                                     "\"held_epochs\": %u, \"max_chi2\": %.2f, \"max_e\": %.2f, "
                                     "\"epoch_fail_t\": %s, \"sat_fail_t\": %s, \"sat_fail_n\": %s, \"held_t\": %s, "
                                     "\"held_n\": %s, \"noise_max\": %.3f, \"all_held\": %u, \"latches\": %u, "
                                     "\"latch_releases\": %u, \"latch_epochs\": %u, ",
                                 sh.cfg_.dv_check, fs.dv_checks, fs.dv_sat_fails, fs.dv_sat_win_fails,
                                 fs.dv_epoch_fails, fs.dv_epoch_win_fails, fs.dv_held_rr, fs.dv_held_epochs,
                                 static_cast<double>(fs.dv_max_chi2), static_cast<double>(fs.dv_max_e),
                                 list(dv_epoch_fail_t).c_str(), list(dv_sat_fail_t).c_str(),
                                 ilist(dv_sat_fail_n).c_str(), list(dv_held_t).c_str(), ilist(dv_held_n).c_str(),
                                 static_cast<double>(fs.dv_noise_max_mps), fs.dv_all_held, fs.dv_latches,
                                 fs.dv_latch_releases, fs.dv_latch_epochs);
#if RAW_REPLAY_DVHDG
                    // the heading allowance for a launch with no magnetometer heading: its setting, and whether
                    // the flight took it (the launch's)
                    std::fprintf(ej, "\"hdg_unset_deg\": %.1f, \"hdg_unset\": %u}, ", sh.cfg_.dv_hdg_unset_deg,
                                 static_cast<unsigned>(fs.dv_hdg_unset));
#else
                    std::fprintf(ej, "\"hdg_unset_deg\": null, \"hdg_unset\": null}, ");
#endif
                }
#endif
#if RAW_REPLAY_ROT
                std::fprintf(ej, "\"rot\": {\"axial_m\": %.3f, \"radial_m\": %.3f, \"min_rps\": %.3f, \"hold_rps\": %.3f, \"hold_s\": %.3f, "
                                 "\"skip_rps\": %.3f, \"skip_s\": %.3f, "
                                 "\"after_apogee\": %d, \"max_sigma_mps\": %.3f, \"held\": %u, \"skipped\": %u}, ",
                             sh.cfg_.rot_lever_axial_m, sh.cfg_.rot_lever_radial_m, sh.cfg_.rot_lever_min_rps,
                             sh.cfg_.att_hold_rate_rps,
                             sh.cfg_.att_hold_s, sh.cfg_.rot_skip_rate_rps, sh.cfg_.rot_skip_s,
                             sh.cfg_.rot_after_apogee ? 1 : 0,
                             static_cast<double>(fs.rot_max_sigma_mps), fs.att_held, fs.rot_skipped);
#endif
#if RAW_REPLAY_ANTENNA
                std::fprintf(ej, "\"antenna\": {\"known\": %d, \"b_m\": [%.4f, %.4f, %.4f], \"sigma_axial_m\": %.4f, "
                                 "\"sigma_radial_m\": %.4f, \"min_rps\": %.3f, \"att_mode\": %u, "
                                 "\"roll_epochs\": %u, \"roll_info\": %.6g, \"roll_hy\": %.6g}, ",
                             sh.cfg_.ekf.lever_on ? 1 : 0, sh.cfg_.ekf.lever_b_m[0], sh.cfg_.ekf.lever_b_m[1],
                             sh.cfg_.ekf.lever_b_m[2], sh.cfg_.antenna_sigma_axial_m, sh.cfg_.antenna_sigma_radial_m,
                             sh.cfg_.antenna_min_rps, static_cast<unsigned>(sh.cfg_.ekf.lever_att_mode),
                             fs.lever_roll_epochs, fs.lever_roll_info, fs.lever_roll_hy);
#endif
#if RAW_REPLAY_WINDUP
                std::fprintf(ej, "\"windup\": {\"k\": %.4f, \"geom\": %d, \"monitor\": %d, \"att_exact\": %d, "
                                 "\"dv_lever_mode\": %d, \"dv_lever_radial_m\": %.3f, \"epochs\": %u, \"misses\": %u, "
                                 "\"cr_misses\": %u, \"max_l1_mps\": %.4f, \"mon_epochs\": %u, \"mon_max_z\": %.3f, "
                                 "\"mon_dp_rps\": %.4f, \"mon_dp_sig_rps\": %.4f}, ",
                             sh.cfg_.windup_k, sh.cfg_.windup_geom ? 1 : 0, sh.cfg_.windup_monitor ? 1 : 0,
                             sh.cfg_.ekf.att_exact ? 1 : 0, static_cast<int>(sh.cfg_.dv_lever_mode),
                             sh.cfg_.dv_lever_radial_m, fs.windup_epochs, fs.windup_misses, fs.windup_cr_misses,
                             static_cast<double>(fs.windup_max_l1_mps), fs.wumon_epochs,
                             static_cast<double>(fs.wumon_max_z), static_cast<double>(fs.wumon_dp_rps),
                             static_cast<double>(fs.wumon_dp_sig_rps));
#endif
                std::fprintf(ej, "\"screen_compare\": [");
                for (size_t i = 0; i < cmps.size(); ++i)
                {
                    const ScreenCmp& c = cmps[i];
                    std::fprintf(ej, "%s{\"tow\": %.3f, \"t\": %.3f, \"g\": %.2f, \"apogee\": %d, \"fw_screened\": %d, "
                                     "\"fw_rr\": %d, \"fw_refused\": %d, \"fw_drop\": %d, \"seq_used\": %d, \"seq_rej\": %d, "
                                     "\"seq_drop\": %d, \"meas\": %d, \"seq_used_pr\": %d, \"seq_rej_pr\": %d, "
                                     "\"fw_dry_used\": %d, \"fw_dry_rej\": %d}",
                                 i ? ", " : "", c.tow, c.t_flight, c.g, c.apogee ? 1 : 0, c.fw_screened ? 1 : 0, c.fw_rr,
                                 c.fw_refused, c.fw_drop ? 1 : 0, c.seq_used, c.seq_rej, c.seq_drop ? 1 : 0, c.meas,
                                 c.seq_used_pr, c.seq_rej_pr, c.fw_dry_used, c.fw_dry_rej);
                }
                std::fprintf(ej, "], ");
            }
#endif
#if RAW_REPLAY_MOSAIC
            if (mosaic || use_votes || cold || !a.rinex.empty() || a.unaided)
            {
                // flight times (s from the ignition edge), null where it never happened
                auto ft = [&](double t_mcu) {
                    return std::isnan(t_mcu) ? std::string("null") : std::to_string(t_mcu + a.t_base_s);
                };
                auto edges = [&](const rr::FlightVotes::Edges& e) {
                    std::string o = "{\"on\": [";
                    for (size_t i = 0; i < e.on.size(); ++i)
                        o += (i ? ", " : "") + ft(e.on[i]);
                    o += "], \"off\": [";
                    for (size_t i = 0; i < e.off.size(); ++i)
                        o += (i ? ", " : "") + ft(e.off[i]);
                    return o + "]}";
                };
                auto num = [](double x) { return std::isfinite(x) ? std::to_string(x) : std::string("null"); };
                const ShadowMeasStats& ms = stage.stats();
                const rr::MosaicFeed::Counts& fc = feed.counts();
                std::fprintf(ej, "\"receiver\": \"%s\", \"cold_start\": %d, \"unaided\": %d, \"sat_cap\": %d, "
                                 "\"fuse_ms\": %.0f, ",
                             a.receiver.c_str(), cold ? 1 : 0, a.unaided ? 1 : 0, cfg.sat_cap,
                             fusedEpochPeriod(rx) * 1000.0);
#if RAW_REPLAY_BUILD_LITE
                std::fprintf(ej, "\"build_lite\": %d, ", rx.build_lite ? 1 : 0);
#endif
                if (!in.xep.empty())
                    std::fprintf(ej, "\"sim\": {\"epochs\": %u, \"built\": %u, \"eph_standins\": %u, "
                                     "\"below_cn0\": %u, \"below_mask\": %u, \"meas_built\": %u}, ",
                                 sim_counts.epochs, sim_counts.built, sim_counts.eph_standins, sim_counts.below_cn0,
                                 sim_counts.below_mask, sim_counts.meas_built);
#if RAW_REPLAY_DOPAVG
                if (rx.rr_pair_avg || sim_grid)
                {
                    // the Dopplers in pairs: the driver's counts (the sim's where it has epochs), the filter's lag for
                    // an epoch as it came and the more for one in pairs, and each grid epoch with a Doppler not averaged
                    // (flight time, usable, averaged: as it came, or in pairs with the rest dropped)
                    const mosaic::MosaicDopplerPairStats& ps = in.xep.empty() ? feed.pairStats() : sim_pair_st;
                    std::string al;
                    char b[64];
                    for (size_t i = 0; i < pair_alone_at.size(); ++i)
                    {
                        std::snprintf(b, sizeof(b), "%s[%.3f, %.0f, %.0f]", i ? ", " : "", pair_alone_at[i][0],
                                      pair_alone_at[i][1], pair_alone_at[i][2]);
                        al += b;
                    }
                    std::fprintf(ej, "\"pairs\": {\"on\": %d, \"rr_lag_s\": %.4f, \"rr_pair_lag_s\": %.4f, "
                                     "\"max_diff_mps\": %.2f, \"sigma_scale\": %.4f, \"sim_off_grid\": %u, \"epochs\": %u, "
                                     "\"epochs_no_prev\": %u, \"epochs_jump\": %u, \"epochs_few\": %u, \"dopplers\": %u, "
                                     "\"pairs\": %u, \"alone_epoch\": %u, \"dropped\": %u, \"alone_mate\": %u, "
                                     "\"alone_cn0\": %u, \"alone_lock\": %u, \"alone_diff\": %u, \"diff_max_mps\": %.3f, "
                                     "\"alone_at\": [%s]}, ",
                                 rx.rr_pair_avg ? 1 : 0, cfg.ekf.rr_lag_s, cfg.ekf.rr_pair_lag_s,
                                 fcfg.pair_cfg.max_diff_mps, a.rr_pair_sigma > 0.0 ? a.rr_pair_sigma : 1.0, sim_off_grid,
                                 ps.epochs, ps.epochs_no_prev, ps.epochs_jump, ps.epochs_few, ps.dopplers, ps.pairs,
                                 ps.alone_epoch, ps.alone_mate + ps.alone_cn0 + ps.alone_lock + ps.alone_diff,
                                 ps.alone_mate, ps.alone_cn0, ps.alone_lock, ps.alone_diff,
                                 static_cast<double>(ps.diff_max_mps), al.c_str());
                }
#endif
                {
                    // each stop at the first row after it (flight time) with RawGnssShadow's reason: 1 sim,
                    // 2 gap, 3 not finite, 4 far from the seed on the pad
                    std::string so;
                    for (size_t i = 0; i < stop_at.size(); ++i)
                        so += (i ? ", " : "") + std::string("[") + std::to_string(stop_at[i].first) + ", " +
                              std::to_string(stop_at[i].second) + "]";
                    std::fprintf(ej, "\"filter\": {\"starts\": %u, \"stops\": %u, \"last_stop\": %u, \"stop_at\": [%s]}, ",
                                 sh.stats().starts, sh.stats().stops, static_cast<unsigned>(sh.stats().last_stop),
                                 so.c_str());
                }
#if RAW_REPLAY_HDGOBS
                if (cfg.ekf.hdg_obs_mode != 0)
                {
                    // the experiment's settings, and each closed epoch's share of the heading (flight time, s)
                    std::string hs;
                    char b[48];
                    for (size_t i = 0; i < hdg_obs_at.size(); ++i)
                    {
                        std::snprintf(b, sizeof(b), "%s[%.3f, %.4f]", i ? ", " : "", hdg_obs_at[i].first,
                                      static_cast<double>(hdg_obs_at[i].second));
                        hs += b;
                    }
                    std::fprintf(ej, "\"hdg_obs\": {\"mode\": %d, \"scope\": %d, \"fh_mps2\": %.3f, \"tau_s\": %.3f, "
                                     "\"sigma_v\": %.3f, \"window_s\": %.3f, \"open_g\": %.3f, \"open_hold_s\": %.3f, "
                                     "\"floor\": %.3f, \"s_at\": [%s]}, ",
                                 cfg.ekf.hdg_obs_mode, cfg.heading_obs_scope, cfg.ekf.hdg_obs_fh_mps2,
                                 cfg.ekf.hdg_obs_tau_s, cfg.ekf.hdg_obs_sigma_v, cfg.ekf.hdg_obs_window_s,
                                 cfg.ekf.hdg_obs_open_g, cfg.ekf.hdg_obs_open_hold_s, cfg.ekf.hdg_obs_floor, hs.c_str());
                }
#endif
                if (gnss_only.asked())
                    std::fprintf(ej, "\"gnss_only\": {\"from_s\": %.3f, \"gate\": \"%s\", \"launches\": %d, "
                                     "\"prior_deg\": %.1f, \"tilt_floor_deg\": %.2f}, ",
                                 a.gnss_only_from_s,
                                 gnss_only.gate() == rr::GnssOnlyArm::kGateLaunch ? "launch" : "ground",
                                 gnss_only.launches(), gnss_only.priorDeg(), sh.cfg_.heading_launch_tilt_sigma_deg);
                if (a.given.count("--corrupt-at-s"))
                {
                    // the corrupted epoch: its satellites' errors as injected, and how many of them the stage built
                    // (the cap keeps 16 of the sky); what the filter did with them is the epochs file's
                    std::string cs;
                    for (size_t i = 0; i < corrupt_sats.size(); ++i)
                        cs += (i ? ", " : "") + std::string("[") + std::to_string(corrupt_sats[i].sys) + ", " +
                              std::to_string(corrupt_sats[i].prn) + ", " + std::to_string(corrupt_sats[i].d_rr) +
                              ", " + std::to_string(corrupt_sats[i].d_pr) + "]";
                    std::fprintf(ej, "\"corrupt\": {\"t_s\": %s, \"tow_ms\": %lld, \"pr_epoch_asked\": %d, "
                                     "\"sats\": [%s], \"built_rr\": %d, \"built_pr\": %d}, ",
                                 corrupt_tow_ms >= 0 ? std::to_string(corrupt_t_s).c_str() : "null",
                                 static_cast<long long>(corrupt_tow_ms), a.corrupt_pr_epoch ? 1 : 0, cs.c_str(),
                                 corrupt_built_rr, corrupt_built_pr);
                }
#if RAW_REPLAY_CAPGOV
                {
                    // the satellite cap's governor (--cap-max): what moved it
                    const CapGovernor::Stats& gs = sh.capGovernor().stats();
                    std::fprintf(ej, "\"cap_gov\": {\"on\": %d, \"floor\": %d, \"ceiling\": %d, \"cap\": %d, "
                                     "\"closes\": %u, \"raises\": %u, \"cuts\": %u, \"floors\": %u, \"late\": %u, "
                                     "\"lost\": %u, \"boosts\": %u, \"trims\": %u, \"trimmed\": %u, "
                                     "\"slice_us\": %.0f}, ",
                                 sh.capGovernor().on() ? 1 : 0, sh.capGovernor().config().floor,
                                 sh.capGovernor().config().ceiling, sh.capGovernor().cap(), gs.closes, gs.raises,
                                 gs.cuts, gs.floors, gs.late, gs.lost, gs.boosts, gs.trims, gs.trimmed,
                                 cfg.epoch_slice_us);
                }
#endif
                std::fprintf(ej, "\"stage\": {\"epochs\": %u, \"untimed\": %u, \"idle\": %u, \"built\": %u, "
                                 "\"eph_added\": %u, \"eph_bad\": %u, \"eph_sats\": %d, \"tracked\": %d, "
                                 "\"tracked_no_eph\": %d, \"capped_out\": %u}, ",
                             ms.epochs, ms.untimed, ms.idle, ms.built, ms.eph_added, ms.eph_bad, ms.eph_sats,
                             ms.tracked, ms.tracked_no_eph, ms.capped_out);
                std::fprintf(ej, "\"sbf\": {\"meas\": %u, \"meas_bad\": %u, \"off_grid\": %u, \"empty\": %u, "
                                 "\"epochs\": %u, \"dtr_anchored\": %u, \"dtr_zero\": %u, \"clock_jumps\": %u, "
                                 "\"rebases\": %u, \"pvt\": %u, \"pvt_fix\": %u, \"nav\": %u, \"nav_ok\": %u, "
                                 "\"nav_dropped\": %u, \"nav_bad\": %u, \"ion\": %u, \"ion_applied\": %u, "
                                 "\"crc_fails\": %u}, ",
                             fc.meas, fc.meas_bad, fc.off_grid, fc.empty, fc.epochs, fc.dtr_anchored, fc.dtr_zero,
                             fc.jumps, fc.rebases, fc.pvt, fc.pvt_fix, fc.nav, fc.nav_ok, fc.nav_dropped, fc.nav_bad,
                             fc.ion, fc.ion_applied, sbfp->crcFails());
                std::fprintf(ej, "\"eph_from\": {\"log\": %d, \"preloaded\": %d, \"preloaded_filed\": %d, "
                                 "\"preloaded_unfiled\": %d, \"rinex\": %d, \"rinex_stale\": %d, \"events\": %d}, ",
                             eph_nav, eph_pre, eph_pre_filed, static_cast<int>(nav_pre.size() - nav_pre_next), eph_rinex,
                             rinex_stale, eph_n);
                // the driver's time tag: why epochs were left untimed (the log's first ReceiverTime: -1 none, else
                // its FINETIME)
                {
                    const mosaic::MosaicTimeTag::Stats& ts = feed.timeTag().stats();
                    using Brk = mosaic::MosaicTimeTag::Break;
                    const rr::MosaicFeed::Counts& tc = feed.counts();
                    std::fprintf(ej, "\"time_tag\": {\"receiver_time_in_log\": %d, \"from_boot\": %d, "
                                     "\"assume_finetime\": %d, \"untimed_not_finetime\": %d, \"untimed_pps_stale\": %d, "
                                     "\"untimed_no_line\": %d, \"untimed_no_bias\": %d, \"untimed_edge_off\": %d, "
                                     "\"untimed_off_chain\": %d, \"untimed_held\": %d, \"breaks_first\": %u, "
                                     "\"breaks_reset\": %u, \"breaks_rebase\": %u, \"breaks_expired\": %u, "
                                     "\"breaks_bias_step\": %u, \"breaks_check_failed\": %u, \"breaks_line_restart\": %u, "
                                     "\"breaks_off_chain\": %u, \"confirmed\": %u, \"edges_off\": %u, "
                                     "\"edges_off_chain\": %u, \"refs_dropped\": %u, \"jumps_resolved\": %u, "
                                     "\"clock_resets\": %u, \"k_unpaired\": %d, \"k_lost\": %d, \"pps_judged\": %u, "
                                     "\"pps_max_us\": %d, \"pps_over\": %u, \"pps_held\": %u}, ",
                                 first_rt, a.time_tag_from_boot ? 1 : 0, a.assume_finetime ? 1 : 0, untimed_not_fine,
                                 untimed_pps_stale, untimed_no_line, untimed_no_bias, untimed_edge_off,
                                 untimed_off_chain, untimed_held, ts.breaks[static_cast<int>(Brk::FIRST)],
                                 ts.breaks[static_cast<int>(Brk::RESET)], ts.breaks[static_cast<int>(Brk::REBASE)],
                                 ts.breaks[static_cast<int>(Brk::EXPIRED)], ts.breaks[static_cast<int>(Brk::BIAS_STEP)],
                                 ts.breaks[static_cast<int>(Brk::CHECK_FAILED)],
                                 ts.breaks[static_cast<int>(Brk::LINE_RESTART)],
                                 ts.breaks[static_cast<int>(Brk::OFF_CHAIN)], ts.confirmed, ts.edges_off,
                                 ts.edges_off_chain, ts.refs_dropped, tc.resolved, tc.resets, k_unpaired, k_lost,
                                 ts.pps.checks, ts.pps.max_us, ts.pps.over, ts.pps.held);
                }
                // the faults' clock, the pad's zero-velocity updates (with the levelling they hold a static log's
                // velocity unless --gnss-only-from-s), each closed epoch's Dopplers and largest NIS, and the
                // magnetometer
                auto median = [](std::vector<double> v) {
                    if (v.empty())
                        return std::string("null");
                    std::sort(v.begin(), v.end());
                    return std::to_string(v[v.size() / 2]);
                };
                auto p95 = [](std::vector<double> v) {
                    if (v.empty())
                        return std::string("null");
                    std::sort(v.begin(), v.end());
                    return std::to_string(v[std::min(v.size() - 1, static_cast<size_t>(0.95 * v.size()))]);
                };
                const RawGnssShadow::Stats& fs = sh.stats();
                std::fprintf(ej, "\"faults_clock\": \"%s\", \"pad_zupt\": %d, \"pad_zupts\": %u, \"landed_zupts\": %u, ",
                             faults_on_flight_time ? "flight_time" : "tow", sh.cfg_.pad_zupt ? 1 : 0, fs.pad_zupts,
                             fs.zupts);
                std::fprintf(ej, "\"doppler\": {\"epochs\": %zu, \"used_rr_median\": %s, \"rejected_rr\": %d, "
                                 "\"rejected_rr_epochs\": %d, \"nis_max_median\": %s, \"nis_max_p95\": %s}, ",
                             dop_used.size(), median(std::vector<double>(dop_used.begin(), dop_used.end())).c_str(),
                             dop_rejected, dop_rejected_epochs,
                             median(std::vector<double>(dop_nis.begin(), dop_nis.end())).c_str(),
                             p95(std::vector<double>(dop_nis.begin(), dop_nis.end())).c_str());
                std::fprintf(ej, "\"mag\": {\"handed\": %d, \"fused\": %u, \"uncal\": %u, \"suspect\": %u, "
                                 "\"dip\": %u, \"norm_median_ut\": %s, \"model_ut\": %s}, ",
                             mag_handed, fs.mag_fused, fs.mag_uncal, fs.mag_suspect, fs.mag_dip,
                             median(mag_norms).c_str(), num(model_field_ut > 0.0 ? model_field_ut : NAN).c_str());
                if (use_votes)
                {
                    const rr::FlightVotes::Report& r = votes.report();
                    std::fprintf(ej, "\"quorum\": {\"flight_state\": \"%s\", \"landed_t\": %s, ",
                                 state_from_quorum ? "quorum" : "log", ft(r.landed_s).c_str());
                    std::fprintf(ej, "\"guard_applied\": %d, \"launch_t\": %s, \"kc_launch_t\": %s, "
                                     "\"burnout_t\": %s, \"gnss_admitted_t\": %s, \"gnss_admitted_reason\": %d, "
                                     "\"guard_latch_t\": %s, \"guard_vz\": %s, \"guard_baro_rate\": %s, "
                                     "\"apogee_t\": %s, \"backstop_t\": %s, ",
                                 vcfg.guard ? 1 : 0, ft(r.launch_s).c_str(), ft(r.kc_launch_s).c_str(),
                                 ft(r.burnout_s).c_str(), ft(r.admitted_s).c_str(), r.admitted_reason,
                                 ft(r.guard_s).c_str(), num(r.guard_vz).c_str(), num(r.guard_baro_rate).c_str(),
                                 ft(r.apogee_s).c_str(), ft(r.backstop_s).c_str());
                    std::fprintf(ej, "\"votes\": {\"vel\": %s, \"baro\": %s, \"gps\": %s, \"pitch\": %s}, ",
                                 edges(r.vel).c_str(), edges(r.baro).c_str(), edges(r.gps).c_str(),
                                 edges(r.pitch).c_str());
                    std::fprintf(ej, "\"at_call\": {\"vel\": %d, \"baro\": %d, \"gps\": %d, \"pitch\": %d, "
                                     "\"filter_votes\": %d, \"baro_available\": %d, \"gps_available\": %d, "
                                     "\"vz\": %s, \"pitch_deg\": %s, \"baro_rate\": %s, \"gps_vu\": %s, \"agl\": %s}, ",
                                 r.call_vel ? 1 : 0, r.call_baro ? 1 : 0, r.call_gps ? 1 : 0, r.call_pitch ? 1 : 0,
                                 r.call_ekf_healthy ? 1 : 0, r.call_baro_ok ? 1 : 0, r.call_gps_ok ? 1 : 0,
                                 num(r.call_vz).c_str(), num(r.call_pitch_deg).c_str(), num(r.call_baro_rate).c_str(),
                                 num(r.call_gps_vu).c_str(), num(r.call_agl).c_str());
                    std::fprintf(ej, "\"passes\": %u, \"fixes\": %u, \"ref_count\": %u, \"ref_from_nav\": %d, "
                                     "\"mach_lockouts\": %u}, ",
                                 r.passes, r.fixes, r.ref_count, r.ref_from_nav ? 1 : 0, r.mach_lockouts);
                    if (tvotesp)
                    {
                        // the same quorum on the truth (--truth-nav): its launch, admission, call and votes
                        const rr::FlightVotes::Report& t = tvotesp->report();
                        std::fprintf(ej, "\"truth_quorum\": {\"launch_t\": %s, \"burnout_t\": %s, "
                                         "\"gnss_admitted_t\": %s, \"guard_latch_t\": %s, \"apogee_t\": %s, "
                                         "\"backstop_t\": %s, \"landed_t\": %s, ",
                                     ft(t.launch_s).c_str(), ft(t.burnout_s).c_str(), ft(t.admitted_s).c_str(),
                                     ft(t.guard_s).c_str(), ft(t.apogee_s).c_str(), ft(t.backstop_s).c_str(),
                                     ft(t.landed_s).c_str());
                        std::fprintf(ej, "\"votes\": {\"vel\": %s, \"baro\": %s, \"gps\": %s, \"pitch\": %s}, ",
                                     edges(t.vel).c_str(), edges(t.baro).c_str(), edges(t.gps).c_str(),
                                     edges(t.pitch).c_str());
                        std::fprintf(ej, "\"at_call\": {\"vel\": %d, \"baro\": %d, \"gps\": %d, \"pitch\": %d}, "
                                         "\"rows\": %zu}, ",
                                     t.call_vel ? 1 : 0, t.call_baro ? 1 : 0, t.call_gps ? 1 : 0, t.call_pitch ? 1 : 0,
                                     truth_nav.size());
                    }
                }
            }
#endif
            bool legacy_votes = true;
#if RAW_REPLAY_MOSAIC
            // with the quorum, its own voters (quorum.votes) say when each voted: the older first-vote times, from
            // --burnout-s and simplified thresholds, are left out so they cannot be read for them
            legacy_votes = !use_votes;
#endif
            if (legacy_votes)
                std::fprintf(ej, "\"pitch_vote_t\": %s, \"vel_vote_t\": %s, \"gate_switch_t\": %s}\n",
                             pitch_vote_t > -1e8 ? std::to_string(pitch_vote_t).c_str() : "null",
                             vel_vote_t > -1e8 ? std::to_string(vel_vote_t).c_str() : "null",
                             gate_switched ? std::to_string(gate_switch_us * 1e-6 + a.t_base_s).c_str() : "null");
            else
                std::fprintf(ej, "\"gate_switch_t\": %s}\n",
                             gate_switched ? std::to_string(gate_switch_us * 1e-6 + a.t_base_s).c_str() : "null");
            std::fclose(ej);
        }
    }
    const RawGnssShadow::Stats& st = sh.stats();
    (void)st;
    std::fprintf(stderr,
                 "%s: %d rows; started %d (restored %d, re-anchored %d); epochs decoded %d (untimed %d, timed "
                 "without a clock line %d, on-board age matched %d), applied %u, stale %u; ephemerides %d pad + %d "
                 "in flight; stops %u (last %u); heading handoffs %u; carrier built %u in %u used %u; non-finite rows "
                 "%d, unhealthy rows %d\n",
                 a.out.c_str(), rows, started ? 1 : 0, restored ? 1 : 0, reanchored ? 1 : 0, epochs_decoded,
                 epochs_untimed, epochs_line_fallback, ages_matched, st.epochs_applied, st.epochs_stale, eph_pad, eph_flight, st.stops,
                 st.last_stop,
                 st.heading_handoffs, CR(st.cr_built), CR(st.cr_in), CR(st.cr_used), nonfinite_rows,
                 unhealthy_rows);
#if RAW_REPLAY_MOSAIC
    if (cold && !seed_ready)
        std::fprintf(stderr, "WARNING: the cold start never had a position (no PVTGeodetic with a solution, no META "
                             "seed_lla): the filter never started\n");
    if (mosaic)
    {
        const rr::MosaicFeed::Counts& fc = feed.counts();
        const ShadowMeasStats& ms = stage.stats();
        std::fprintf(stderr,
                     "  mosaic: MeasEpoch %u (fused grid %u, off it %u, empty %u, refused %u; tagged with a PVT's "
                     "bias %u, without %u), clock jumps %u; PVT %u (fixes %u); nav blocks %u (kept %u, dropped %u, "
                     "bad %u); ionosphere sets applied %u; ephemerides filed: log %d, RINEX %d (stale %d), events "
                     "%d; eph_sats %d, tracked %d without one %d, capped out %u\n",
                     fc.meas, fc.epochs, fc.off_grid, fc.empty, fc.meas_bad, fc.dtr_anchored, fc.dtr_zero, fc.jumps,
                     fc.pvt, fc.pvt_fix, fc.nav, fc.nav_ok, fc.nav_dropped, fc.nav_bad, fc.ion_applied, eph_nav,
                     eph_rinex, rinex_stale, eph_n, ms.eph_sats, ms.tracked, ms.tracked_no_eph, ms.capped_out);
    }
    if (use_votes)
    {
        const rr::FlightVotes::Report& r = votes.report();
        auto ft = [&](double t) { return std::isnan(t) ? -1.0e9 : t + a.t_base_s; };
        std::fprintf(stderr,
                     "  quorum: launch T%+.3f, burnout T%+.3f, GNSS admitted T%+.3f, guard latched T%+.3f (%s), "
                     "apogee call T%+.3f (vel %d baro %d gps %d pitch %d), backstop T%+.3f  [-1e9: never]\n",
                     ft(r.launch_s), ft(r.burnout_s), ft(r.admitted_s), ft(r.guard_s),
                     vcfg.guard ? "applied" : "observed", ft(r.apogee_s), r.call_vel ? 1 : 0, r.call_baro ? 1 : 0,
                     r.call_gps ? 1 : 0, r.call_pitch ? 1 : 0, ft(r.backstop_s));
        if (state_from_quorum)
            std::fprintf(stderr, "  flight state from the quorum: INFLIGHT T%+.3f, LANDED T%+.3f  [-1e9: never]\n",
                         ft(r.launch_s), ft(r.landed_s));
        if (tvotesp)
        {
            const rr::FlightVotes::Report& t = tvotesp->report();
            std::fprintf(stderr, "  quorum on the truth (--truth-nav): apogee call T%+.3f (vel %d baro %d gps %d pitch %d), "
                                 "guard would latch T%+.3f  [-1e9: never]\n",
                         ft(t.apogee_s), t.call_vel ? 1 : 0, t.call_baro ? 1 : 0, t.call_gps ? 1 : 0,
                         t.call_pitch ? 1 : 0, ft(t.guard_s));
        }
    }
#endif
    // A fault asked for that never happened is a test that tested nothing: say so. On a mosaic or cold-start
    // replay (faults on flight time) it fails the run.
    int rc = 0;
    {
        const bool cut_asked = a.given.count("--cut-from-s") || a.given.count("--cut-to-s");
        const bool corrupt_asked = a.given.count("--corrupt-at-s") != 0;
        const char* clock = faults_on_flight_time ? "flight time (MCU s + t_base_s)" : "GPS time of week less --truth-t0-tow";
        if (cut_asked && epochs_cut == 0)
            std::fprintf(stderr, "WARNING: --cut-from-s %.3f --cut-to-s %.3f cut no epoch (on %s)\n", a.cut_from_s,
                         a.cut_to_s, clock);
        if (corrupt_asked && epochs_corrupted_sats == 0)
            std::fprintf(stderr, "WARNING: --corrupt-at-s %.3f corrupted no satellite (on %s)\n", a.corrupt_at_s, clock);
        if (faults_on_flight_time &&
            ((cut_asked && epochs_cut == 0) || (corrupt_asked && epochs_corrupted_sats == 0)))
            rc = 3;
    }
#if RAW_REPLAY_MOSAIC
    if (mosaic || cold)
    {
        if (untimed_not_fine + untimed_pps_stale + untimed_no_line + untimed_no_bias + untimed_edge_off +
                untimed_off_chain + untimed_held > 0)
            std::fprintf(stderr, "  time tag: untimed %d not FINETIME (ReceiverTime %s), %d with no 1PPS edge paired "
                                 "within 120 s, %d with no valid clock line, %d with no PVT bias to carry, %d with the "
                                 "last edge off the line, %d with it off the chain, %d held (the chain of biases not "
                                 "yet confirmed)\n",
                         untimed_not_fine, first_rt < 0 ? "never in the log" : "in the log", untimed_pps_stale,
                         untimed_no_line, untimed_no_bias, untimed_edge_off, untimed_off_chain, untimed_held);
        if (mosaic)
        {
            const mosaic::MosaicTimeTag::Stats& ts = feed.timeTag().stats();
            using Brk = mosaic::MosaicTimeTag::Break;
            std::fprintf(stderr, "  time tag chain: breaks %u first, %u receiver clock lost, %u jump count restart, "
                                 "%u expired, %u bias step (last %+.3f ms), %u check failed, %u line restart, %u edge "
                                 "off the chain; %u confirmed by the 1PPS; %u edges off the line, %u off the chain, "
                                 "%u references dropped; jumps placed past 256 ms %u; PPS check judged %u (max %d us, "
                                 "%u over 200 us), held back %u\n",
                         ts.breaks[static_cast<int>(Brk::FIRST)], ts.breaks[static_cast<int>(Brk::RESET)],
                         ts.breaks[static_cast<int>(Brk::REBASE)], ts.breaks[static_cast<int>(Brk::EXPIRED)],
                         ts.breaks[static_cast<int>(Brk::BIAS_STEP)], ts.last_bias_step_ms,
                         ts.breaks[static_cast<int>(Brk::CHECK_FAILED)], ts.breaks[static_cast<int>(Brk::LINE_RESTART)],
                         ts.breaks[static_cast<int>(Brk::OFF_CHAIN)], ts.confirmed, ts.edges_off, ts.edges_off_chain,
                         ts.refs_dropped, feed.counts().resolved, ts.pps.checks, ts.pps.max_us, ts.pps.over,
                         ts.pps.held);
            if (k_unpaired > 0)
                std::fprintf(stderr, "  clock lines: %d taken with no block pairing their edge seen\n", k_unpaired);
            if (k_lost > 0)
                std::fprintf(stderr, "WARNING: %d clock records (0x9D) missing by their seq: the driver's queue was "
                                     "full or the log lost them; edges the driver judged are not all here\n", k_lost);
            else if (k_lost < 0)
                std::fprintf(stderr, "  clock lines carry no seq (an older events file): lost records not counted\n");
        }
        if (first_rt < 0 && epochs_decoded > 0)
            std::fprintf(stderr, "WARNING: no ReceiverTime in the log: the driver times nothing without FINETIME%s\n",
                         a.assume_finetime ? " (--assume-finetime: timed all the same)" : " (--assume-finetime to time them)");
        if (eph_pre > 0)
            std::fprintf(stderr, "  preloaded nav: %d decoded, %d filed when a receiver had them, %d never due in the "
                                 "log (filed where logged)\n",
                         eph_pre, eph_pre_filed, static_cast<int>(nav_pre.size() - nav_pre_next));
        const RawGnssShadow::Stats& fs = sh.stats();
        if (fs.pad_zupts > 0 && a.gnss_only_from_s >= 1.0e8)
            std::fprintf(stderr, "  pad ZUPTs %u: they and the levelling hold a static velocity whatever the receiver "
                                 "gives (--gnss-only-from-s scores GNSS alone)\n",
                         fs.pad_zupts);
        if (mag_handed > 0 && fs.mag_fused == 0)
        {
            std::vector<double> mn = mag_norms;
            std::sort(mn.begin(), mn.end());
            std::fprintf(stderr, "WARNING: %d magnetometer samples, none fused (uncalibrated %u, off the model's size "
                                 "%u, off its dip %u; |B| median %.1f uT, the model's %.1f): the heading is the seed's "
                                 "and the gyro's, and --mag-error-deg %s\n",
                         mag_handed, fs.mag_uncal, fs.mag_suspect, fs.mag_dip, mn[mn.size() / 2], model_field_ut,
                         cold ? "changes nothing here" : "turns only the start's heading");
        }
    }
    if (use_votes)
    {
        // a voter that never existed leaves a 2- or 3-of-N quorum with another latency: say so (and fail a mosaic run)
        const rr::FlightVotes::Report& r = votes.report();
        bool missing = false;
        if (r.fixes == 0)
        {
            std::fprintf(stderr, "WARNING: the quorum had no receiver fix (no PVTGeodetic, no V lines): the GPS voter "
                                 "and GnssAscentGate's admission never existed\n");
            missing = true;
        }
        if (quorum_baro == 0)
        {
            std::fprintf(stderr, "WARNING: the quorum had no barometer (%s): the barometer voter never existed\n",
                         p_ground > 0.0 ? "no B lines" : "no META ground_pressure_pa");
            missing = true;
        }
        if (missing && mosaic)
            rc = 3;
    }
#endif
    return rc;
}
