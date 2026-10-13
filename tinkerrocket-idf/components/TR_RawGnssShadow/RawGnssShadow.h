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
// pad, against the launch site (PadSurvey.h), and frozen at liftoff. The
// choice is made on the pad and kept through the flight; the survey runs
// either way, and starts over when the choice changes.
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
#include "PadSurvey.h"
#include "RawMeas.h"
#include "RawMeasBuilder.h"
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
    };

    // What takes the ionosphere out (the pad status message reports it).
    enum IonoSource : uint8_t
    {
        kIonoNone = 0,       // neither: Klobuchar's night-time 5 ns only
        kIonoBroadcast = 1,  // the GPS broadcast parameters a pad calibration brought
        kIonoPad = 2,        // the pad survey's corrections (some in use)
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
    void onEpoch(const TimedRawEpoch& e);
    // One flight-filter IMU tick; time64_us is its time widened to 64 bits
    // (unwrap32 against the clock it was stamped with).
    void onImu(const ShadowImu& s, int64_t time64_us);
    // A RAW_GNSS_EKF_MSG record, every log_period_s of IMU time while it runs.
    bool takeLog(RawGnssEkfData& out);

    bool running() const { return running_; }
    const Stats& stats() const { return st_; }
    const RawGnssEKF<float>& filter() const { return ekf_; }
    const EphemerisStore& ephemerides() const { return store_; }
    int ephemerisSatellites() const { return eph_sats_; }
    const PadSurvey& survey() const { return survey_; }
    const RawMeasBuilder& builder() const { return builder_; }
    // The last epoch's measurements as the filter took them (pad corrections
    // applied when they were in use), n of them: the console's per-satellite line.
    const RawMeas* lastMeasurements(int& n) const { n = st_.last_meas; return meas_; }
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
    void apply(const TimedRawEpoch& e, double age, double t, uint8_t imu_flags);
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
