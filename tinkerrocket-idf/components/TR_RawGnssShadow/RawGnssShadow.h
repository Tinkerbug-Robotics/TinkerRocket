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
// It learns the barometer's offset with or without a fix of its own (a flight
// without GNSS has the barometer and the IMU). A seed marked restore -- a
// flight the flight computer restored after a reboot -- starts it in flight,
// the barometer's offset taken from its first sample. Its state goes to the
// flight loop every step (navState, ShadowLink::nav).
//
// Levelling. Where the flight filter takes its accelerometer as gravity (the
// ground, and after apogee) the shadow does too -- in flight only once its
// own velocity has held steady from epoch to epoch and the specific force is
// g's size (LevelGate.h): as a drogue opens, the drag is far off vertical.
//
// The heading from thrust (HeadingFit.h, heading_mode). Near vertical
// nothing the filter fuses sees its heading. The fit is seeded at the last
// still step on the pad (the filter's attitude and gyro bias, and the IMU
// biases as the pad left them), runs on the gyro alone from there, and after
// launch takes each raw epoch's Doppler velocity -- an epoch's work gains a
// step for it, between the checks and the receiver clock -- logging each fit
// (HEADING_FIT_MSG). Observe: that is all. Act: from launch the filter's
// heading has heading_pad_sigma_deg (the magnetometer's, not the pad's
// near-certainty), its tilt at least heading_launch_tilt_sigma_deg if that
// is set (the pad cannot tell its tilt from an accelerometer bias, #1626
// K1), and GNSS reaches its attitude only about the vertical
// (att_gate_vertical); once the fit's 1-sigma is heading_handoff_deg or less,
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
// (its Doppler velocity, then the fit) are each one slice at most.
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
        double   max_bridge_s    = 5.0;       // the navigation filter bridges an IMU hole this long
        double   pad_context_delay_s = 6.0;   // the pad context is written from this long after launch
        bool     external_build = false;      // the epochs come built (onBuilt): a GNSS task builds them
        // the heading from thrust (above)
        uint8_t  heading_mode = 0;            // HeadingMode
        HeadingFit::Config heading_fit;
        double   heading_seed_max_s = 3.0;    // a seed this long before launch is too old (not still at ignition)
        double   heading_fit_max_s  = 60.0;   // epochs taken this long after launch
        double   heading_handoff_deg = 10.0;  // act: the fit's 1-sigma at or under this hands its heading over
        double   heading_handoff_rival = 25.0;   // act: ... unless another heading's valley rises less than this
                                                 //   above the fit's (as its sigma's: 25 is five sigmas')
        int      heading_handoff_min_epochs = 0; // act: ... once the fit holds this many epochs
        double   heading_handoff_apart = 2.0;    // act: ... and only while its heading and the filter's are more
                                                 //   than this many of their combined 1-sigmas apart: the filter
                                                 //   lost, not just unsure (0: whatever the filter's) --
        double   heading_handoff_sure_deg = 3.0; //   unless the fit's 1-sigma is at or under this
        double   heading_pad_sigma_deg = 30.0;   // act: the heading's 1-sigma from launch on
        double   heading_launch_tilt_sigma_deg = 0.0;   // act: and the tilt's at least this from launch
                                                        //   (the pad's is the accelerometer bias's too,
                                                        //   #1626 K1; 0: off)
        double   heading_handoff_vel_mps = 3.0;  // act: the velocity let go to this at the hand-off
    };

    enum HeadingMode : uint8_t
    {
        kHeadingOff = 0,
        kHeadingObserve = 1,   // fit and log
        kHeadingAct = 2,       // and hand the heading over (the filter gated about the vertical)
    };

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
        uint32_t mag_suspect = 0;          //   refused: not the model's size
        uint32_t restores = 0;             // starts in flight from a restored flight
        uint32_t gaps_bridged = 0;         // IMU holes the navigation filter stepped over
        uint32_t pad_contexts = 0;         // launches the pad context was taken at
        // the heading from thrust
        uint32_t heading_seeds = 0;        // still pad steps it was seeded at
        uint32_t heading_flights = 0;      // launches it ran from (its seed fresh enough)
        uint32_t heading_epochs = 0;       // epochs it took
        uint32_t heading_vel_fails = 0;    //   with no Doppler velocity (too few satellites agreeing)
        uint32_t heading_handoffs = 0;     // headings handed over
        uint32_t heading_rivals = 0;       // fits another heading's valley came within heading_handoff_rival of
        float    heading_turn_deg = 0.0f;  //   the last one's turn
        float    heading_handoff_s = 0.0f; //   and when, after launch
        HeadingFit::Result heading_last;   // the last fit
        uint32_t heading_vel_us = 0, heading_vel_us_max = 0;      // an epoch's Doppler velocity
        uint32_t heading_solve_us = 0, heading_solve_us_max = 0;  //   and its fit
        uint32_t heading_step_us_max = 0;  // the fit's share of a step
    };

    using Clock = int64_t (*)();

    RawGnssShadow();
    explicit RawGnssShadow(const Config& cfg);
    // What a GNSS task's stage builds with, for a shadow with this config.
    static RawGnssMeasStage::Config stageConfig(const Config& c);
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
    enum : int { kJobBuild, kJobChecks, kJobHeading, kJobHeadingFit, kJobClock, kJobFuse, kJobClose };
    struct Job
    {
        bool          active = false;
        int           phase = kJobBuild;
        TimedRawEpoch e;                  // built here: the epoch
        double        te = 0.0;           // its instant on the filter's clock
        bool          in_flight = false;
        double        g_peak = 0.0;       // the largest |specific force| / g - 1 since the last epoch
        int           fused = 0;          // measurements fused
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
    bool          restored_ = false;    // started in flight from a restored flight
    uint8_t       start_gen_ = 0;       // the seed's start_gen it started from
    double        last_aided_t_ = -1.0e9;   // raw measurements last applied
    double        last_baro_fused_t_ = -1.0e9;
    void          applyMag(const float acc[3]);
    void          applyBaro(double t, uint8_t imu_flags);
    // the heading from thrust
    HeadingFit    fit_;
    bool          fit_ground_ = true;   // the last step was on the ground
    bool          fit_flight_ = false;  // launched with a fresh seed: epochs are taken
    double        fit_launch_t_ = 0.0;
    bool          handed_ = false;      // the heading handed over this flight
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
