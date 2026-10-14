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
// has passed that instant, then built into measurements (RawMeasBuilder,
// with the ephemerides the receiver has sent so far) and applied with its age:
// the receiver is taken back to the epoch by the IMU's own velocity history,
// however late the epoch arrived. One later than max_epoch_age_s, past that
// history, is dropped. An epoch with no MCU time yet (the time sync wants
// 10 s of fixes) only counts for the satellites' relock ages.
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

#include "EphemerisStore.h"
#include "FixCheck.h"
#include "LevelGate.h"
#include "PadSurvey.h"
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
    };

    using Clock = int64_t (*)();

    RawGnssShadow();
    explicit RawGnssShadow(const Config& cfg);
    void setClock(Clock c) { clock_ = c; }
    const Config& config() const { return cfg_; }

    // Stop; ephemerides and relock history are kept.
    void reset();

    void onSeed(const ShadowSeed& s);
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
    void onEpoch(const TimedRawEpoch& e);
    // One flight-filter IMU tick; time64_us is its time widened to 64 bits
    // (unwrap32 against the clock it was stamped with).
    void onImu(const ShadowImu& s, int64_t time64_us);
    // A RAW_GNSS_EKF_MSG record, every log_period_s of IMU time while it runs.
    bool takeLog(RawGnssEkfData& out);
    // A GNSS_CHECK_MSG record, one per receiver fix judged.
    bool takeCheckLog(GnssCheckData& out);

    bool running() const { return running_; }
    const Stats& stats() const { return st_; }
    const RawGnssEKF<float>& filter() const { return ekf_; }
    const EphemerisStore& ephemerides() const { return store_; }
    int ephemerisSatellites() const { return eph_sats_; }
    const PadSurvey& survey() const { return survey_; }
    const RawMeasBuilder& builder() const { return builder_; }
    const FixCheck& fixCheck() const { return fixcheck_; }
    const SatJumpCheck& satCheck() const { return satcheck_; }
    // The last epoch's measurements as the filter took them (pad corrections
    // applied when they were in use), n of them: the console's per-satellite line.
    // The last epoch's measurements (none while the next one is being built
    // over them).
    const RawMeas* lastMeasurements(int& n) const { n = job_.active ? 0 : st_.last_meas; return meas_; }
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
    void startJob(const TimedRawEpoch& e, double te, uint8_t imu_flags);
    void workJob(double t);
    void jobUnit(double t);
    void jobChecks(double age, double t);
    void closeJob(double age, double t);
    void applyDueFixes(double t);
    void judge(const TimedFix& f, double t_fix);
    void chooseIono(bool in_flight);
    void dropFront();
    void fillLog(uint32_t time_us, uint8_t imu_flags);
    int64_t now() const { return clock_ ? clock_() : 0; }

    Config        cfg_;
    Clock         clock_ = nullptr;
    RawGnssEKF<float> ekf_;
    EphemerisStore store_;
    RawMeasBuilder builder_;
    RawMeas       meas_[RawEpoch::kMaxObs];
    TimedRawEpoch pending_[2];
    // the epoch being worked (startJob .. closeJob), a slice a step
    enum : int { kJobBuild, kJobChecks, kJobClock, kJobFuse, kJobClose };
    struct Job
    {
        bool          active = false;
        int           phase = kJobBuild;
        TimedRawEpoch e;
        double        te = 0.0;           // its instant on the filter's clock
        bool          in_flight = false;
        double        r_prior[3] = {};    // the receiver when it started: the builder's, the survey's
        int           n = 0;              // measurements built
        int           fused = 0;          //   and fused
        RawGnssStats  rs;
        int64_t       t0_us = 0;
        uint32_t      build_us = 0, update_us = 0, slices = 0;
    };
    Job           job_;
    int           npending_ = 0;
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
    uint16_t      week_ = 0;
    double        last_tow_s_ = -1.0;  // the last epoch's time of week (any epoch)
    int           eph_sats_ = 0;
    PadSurvey     survey_;
    double        lock_age_[RawEpoch::kMaxObs] = {};
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
    static constexpr int kFixQ = 4;      // fixes waiting for the IMU to pass them (0.4 s at 10 Hz)
    TimedFix      fixq_[kFixQ] = {};
    int           nfix_ = 0;
    GnssCheckData chk_[kFixQ] = {};      // check records waiting to be taken
    int           nchk_ = 0;
    ShadowIono    iono_in_ = {};
    bool          have_iono_ = false;
    bool          iono_dirty_ = true;   // the builder's model is to be chosen again
    bool          use_broadcast_ = false;
    bool          last_timed_ = false;
    uint8_t       pend_flags_ = 0;      // RGE_STALE / RGE_RESYNC / RGE_CLOCK_STEP since the last record
    RawGnssEkfData log_ = {};
    bool          log_ready_ = false;
    uint64_t      prop_sum_ = 0;        // IMU steps since the last record
    uint32_t      prop_n_ = 0;
    Stats         st_;
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_SHADOW_H
