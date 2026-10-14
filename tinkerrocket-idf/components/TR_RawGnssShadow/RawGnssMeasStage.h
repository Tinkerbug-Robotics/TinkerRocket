// RawGnssMeasStage.h — the raw-GNSS filter's measurement side (#1600, #1626):
// a receiver's raw epoch built into the measurements the filter fuses
// (RawMeasBuilder), with the ephemerides the receiver has sent and the
// ionosphere chosen on the pad.
//
// It needs no filter state: only a rough receiver position (tens of metres
// do: the elevations and the atmosphere) and whether the rocket is in flight.
// So the navigation filter has it run in a task of its own, below the
// filter's priority (main.cpp): the build, about two-thirds of an epoch's
// cost (~2 ms a satellite on the Beetle), runs while the filter's task
// sleeps, and the filter takes the finished epoch (BuiltEpoch) and fuses it a
// slice a step, the IMU, barometer and magnetometer going on in between. The
// shadow keeps a stage of its own and builds in its own job where nothing
// builds for it (RawGnssShadow::Config::external_build): a host test drives
// it that way.
//
// Relocks. The builder keeps each satellite's lock age. The filter's
// satellite check (SatJumpCheck) finds the jumps the receiver did not report,
// and they come back here (markRelock) for the epochs after.
//
// The ionosphere. Chosen for each epoch built on the pad and kept through
// the flight: the GPS broadcast parameters a pad calibration brought while
// they are no older than iono_max_age_days by the receiver's date, else none
// (the filter then takes the pad survey's corrections). A built epoch says
// which, and when the choice changed: the pad survey starts over then.
//
// Pure C++: tests_cpp/test_raw_gnss_shadow.cpp.
#ifndef RAW_GNSS_MEAS_STAGE_H
#define RAW_GNSS_MEAS_STAGE_H

#include <cstddef>
#include <cstdint>

#include "EphemerisStore.h"
#include "RawMeas.h"
#include "RawMeasBuilder.h"
#include "ShadowLink.h"

namespace gnss_raw
{

// One epoch, built: what the GNSS task hands the filter's (a ring in PSRAM,
// main.cpp) and what the shadow's own job builds into.
struct BuiltEpoch
{
    int64_t  mcu_us = 0;            // the epoch's GPS instant on the MCU clock (TimedRawEpoch)
    double   tow_s = 0.0;           // its GPS time of week
    uint16_t gps_week = 0;
    int      n = 0;                 // measurements
    RawMeas  meas[RawEpoch::kMaxObs] = {};
    double   lock_age_s[RawEpoch::kMaxObs] = {};  // each one's satellite's relock age (the pad survey's)
    double   r_approx[3] = {};      // the receiver it was built at, ECEF (0: none)
    uint8_t  use_broadcast = 0;     // the ionosphere taken out with the broadcast parameters
    uint8_t  iono_changed = 0;      //   chosen anew for it: the pad survey starts over
    uint32_t build_us = 0;          // its build (with a clock)
};

class RawGnssMeasStage
{
public:
    struct Config
    {
        RawMeasBuilder::Config meas;
        uint16_t default_week = 2440;    // until an epoch names the GPS week (2026-10)
        int      iono_max_age_days = 30; // broadcast parameters older than this are not used
    };

    using Clock = int64_t (*)();

    RawGnssMeasStage();
    explicit RawGnssMeasStage(const Config& cfg);
    void setClock(Clock c) { clock_ = c; }

    // A broadcast ephemeris (RTCM3 payload): into the store. False if it did
    // not decode.
    bool onEphemeris(const uint8_t* payload, size_t len);
    // The broadcast ionosphere a pad calibration brought (or NVS kept).
    void setIono(const ShadowIono& io);
    // A satellite the filter found jumped at tow: relocked then.
    void markRelock(int sys, int prn, double tow) { builder_.markRelock(sys, prn, tow); }

    // An epoch seen: its week and time of week, and which satellites are
    // tracked without an ephemeris (the console, the pad status).
    void see(const TimedRawEpoch& e);
    // An epoch that will not be built: its satellites' lock ages only.
    void noteEpoch(const RawEpoch& ep) { builder_.noteEpoch(ep); }

    // Build: begin(), then next() until it says done -- max_obs observations
    // a call, as the shadow's own job slices it -- or build() at once. e must
    // outlive the build. r_approx: the receiver, ECEF (nullptr: none, no
    // elevations or atmosphere).
    void begin(const TimedRawEpoch& e, const double* r_approx, bool in_flight, BuiltEpoch& out);
    bool next(BuiltEpoch& out, int max_obs);
    void build(const TimedRawEpoch& e, const double* r_approx, bool in_flight, BuiltEpoch& out);

    // The GNSS task's turn for each epoch the receiver sends: seen, then
    // built (true: out is it, to hand the filter's task) if it has its MCU
    // time and the filter runs (nav: the filter's state, its position the
    // receiver's); otherwise its satellites' lock ages only.
    bool offer(const TimedRawEpoch& e, const ShadowNav& nav, BuiltEpoch& out);
    // ... or, when the handoff to the filter's task is full (the two before
    // it not yet taken), not built: seen, and its lock ages kept.
    void skip(const TimedRawEpoch& e);

    // Days from the ionosphere's file to the receiver's date (negative if the
    // file is dated later); false until both are known.
    bool ionoAgeDays(int& days) const;
    bool useBroadcast() const { return use_broadcast_; }
    int ephemerisSatellites() const { return st_.eph_sats; }
    const EphemerisStore& ephemerides() const { return store_; }
    const RawMeasBuilder& builder() const { return builder_; }
    const ShadowMeasStats& stats() const { return st_; }

private:
    void chooseIono(bool in_flight, BuiltEpoch& out);
    int64_t now() const { return clock_ ? clock_() : 0; }

    Config           cfg_;
    Clock            clock_ = nullptr;
    EphemerisStore   store_;
    RawMeasBuilder   builder_;
    const TimedRawEpoch* e_ = nullptr;   // the epoch being built
    int64_t          t0_us_ = 0;
    uint16_t         week_ = 0;
    double           last_tow_s_ = -1.0; // the last epoch's time of week (any epoch)
    ShadowIono       iono_in_ = {};
    bool             have_iono_ = false;
    bool             iono_dirty_ = true; // the builder's model is to be chosen again
    bool             use_broadcast_ = false;
    ShadowMeasStats  st_ = {};
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_MEAS_STAGE_H
