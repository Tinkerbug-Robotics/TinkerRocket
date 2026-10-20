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
// The satellite cap (Config::sat_cap; 0, none, is the Beetle's). Every
// satellite costs the flight computer a build and a fusion each epoch, and a
// receiver that tracks 30 of them at 10 Hz may ask more than its core has
// to spare, so offer() can build at most sat_cap of them: up to four GPS
// first (the filter's clock is GPS's; Galileo and BeiDou each carry a bias
// against it), then the rest by C/N0, each satellite built the epoch before
// counting 3 dB more so the set does not churn between two of about the same
// strength. Only satellites the builder would build are picked (a
// pseudorange at or above the system's C/N0 floor, an ephemeris); the
// elevation mask it cannot tell before the build, so a pick the build drops
// under it is picked last, after every other, until it is tried again
// kCapMaskRetryS later. The whole epoch is still seen and noted, so
// tracked_no_eph and every satellite's lock age stay those of the sky: a
// satellite left out for a while comes back with its lock age, not as a
// relock. (The carrier range rate, off where the cap is used, would start
// over for it.) stats().capped_out counts the buildable satellites left out.
// Where the filter's cap governor runs (CapGovernor.h), the cap is the one
// the navigation state offer() builds at carries (ShadowNav::sat_cap): the
// same picks with their hold, so a raise adds the strongest satellite left
// out and a cut drops the weakest picked. An epoch built above the stage's
// own sat_cap (the governor's floor) says which of its measurements a build
// at the floor would have made (BuiltEpoch::floor_keep): the cap's first
// sat_cap picks, in the order it picks them. The filter keeps only those of
// an epoch whose job starts in the boost (RawGnssShadow::trimToFloor), so an
// epoch built on the pad at the governor's cap just before the ignition is
// worked as the floor's.
//
// The cheaper build (Config::meas.lite, RawMeasBuilder.h; off unless the
// receiver profile's build_lite sets it): offer() then hands the builder the
// filter's geodetic position as it has it (nav.lla, not worked back out of
// ECEF), looks each observation's ephemeris up once an epoch (see(), for the
// cap and the builder both), and notes the cap's picks' lock ages once. Once
// the filter has taken its receiver clock (nav.clock), the orbits that come
// due at once after a gap are solved a few an epoch, and the epoch built from
// those solved; a pick left out until its solve keeps its place in the cap.
//
// Pure C++: tests_cpp/test_raw_gnss_shadow.cpp, tests_cpp/test_raw_gnss_stage_v9.cpp.
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
    uint8_t  floor_trim = 0;        // built at the cap governor's cap, above the stage's own sat_cap (the floor),
                                    //   with satellites a build at the floor would have left out: floor_keep
    int      n = 0;                 // measurements
    RawMeas  meas[RawEpoch::kMaxObs] = {};
    double   lock_age_s[RawEpoch::kMaxObs] = {};  // each one's satellite's relock age (the pad survey's)
    double   r_approx[3] = {};      // the receiver it was built at, ECEF (0: none)
    uint8_t  use_broadcast = 0;     // the ionosphere taken out with the broadcast parameters
    uint8_t  iono_changed = 0;      //   chosen anew for it: the pad survey starts over
    uint8_t  rr_paired = 0;         // its Dopplers pairs' means (TimedRawEpoch's; in the padding before build_us)
    uint32_t build_us = 0;          // its build (with a clock)
    uint64_t floor_keep = 0;        // with floor_trim, the measurements a build at the floor would have made (bit
                                    //   i: meas[i]): what the filter keeps of it in the boost (RawGnssShadow)
};
static_assert(RawEpoch::kMaxObs <= 64, "BuiltEpoch::floor_keep has a bit a measurement");

class RawGnssMeasStage
{
public:
    struct Config
    {
        RawMeasBuilder::Config meas;
        uint16_t default_week = 2440;    // until an epoch names the GPS week (2026-10)
        int      iono_max_age_days = 30; // broadcast parameters older than this are not used
        int      sat_cap = 0;            // satellites built an epoch at most (offer); 0: all
    };

    static constexpr int kCapGps = 4;          // GPS satellites the cap takes first, where there are any
    static constexpr float kCapHoldDb = 3.0f;  // what a satellite built the epoch before counts more by
    static constexpr double kCapMaskRetryS = 10.0;   // how long a pick the build dropped under the mask is picked last
    // addEphemeris: a toe at most this far from the receiver's time. A GPS
    // toe runs up to 2 h ahead and the store uses one up to 2 h from it, so
    // nothing in use or about to be is refused; a copy weeks old is.
    static constexpr double kEphMaxOffsetS = 4.0 * 3600.0;

    using Clock = int64_t (*)();

    RawGnssMeasStage();
    explicit RawGnssMeasStage(const Config& cfg);
    void setClock(Clock c) { clock_ = c; }

    // A broadcast ephemeris (RTCM3 payload): into the store. False if it did
    // not decode.
    bool onEphemeris(const uint8_t* payload, size_t len);
    // ... or one a driver decoded itself (the mosaic-G5's SBF navigation
    // blocks): the same from there on. False, and counted bad, if it is not
    // valid, not one the builder can place (GPS, Galileo, BeiDou not GEO, PRN
    // 1-63), or, where the last epoch seen named its week, its toe more than
    // kEphMaxOffsetS from that epoch's time.
    bool addEphemeris(const gps_eph_t& e);
    // Whether sys/prn has an ephemeris to be built by at GPS time of week
    // tow_gps, as the builder picks it (BeiDou's in its own time).
    bool hasEphemeris(int sys, int prn, double tow_gps) const;
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
    // receiver's); otherwise its satellites' lock ages only. With sat_cap,
    // built from the cap's picks.
    bool offer(const TimedRawEpoch& e, const ShadowNav& nav, BuiltEpoch& out);
    // ... or, when the handoff to the filter's task is full (the two before
    // it not yet taken), not built: seen, and its lock ages kept.
    void skip(const TimedRawEpoch& e);
    // The cap the next epoch is built with: nav's (the filter's cap
    // governor's) where it gives one and the stage has a cap, else the
    // config's. offer() takes it itself; a caller that builds its own way
    // (the harness's sims) takes it here.
    void useNavCap(const ShadowNav& nav);
    int satCap() const { return cap_; }

    // Days from the ionosphere's file to the receiver's date (negative if the
    // file is dated later); false until both are known.
    bool ionoAgeDays(int& days) const;
    bool useBroadcast() const { return use_broadcast_; }
    int ephemerisSatellites() const { return st_.eph_sats; }
    const EphemerisStore& ephemerides() const { return store_; }
    const RawMeasBuilder& builder() const { return builder_; }
    const ShadowMeasStats& stats() const { return st_; }
#ifdef TR_EPH_CAPACITY
    // Whether the caller's EphemerisStore::kCapacity (pass it), this
    // component's and TR_GNSSRaw's (EphemerisStore::builtCapacity) are one:
    // the stage holds the store, so each lays it out by its own. A build that
    // sets TR_EPH_CAPACITY starts no stage unless they are.
    static bool ephCapacityAgrees(size_t caller_capacity);
#endif

private:
    static constexpr int kPrns = 64;    // PRNs a system, 0-63, as the builder keeps them

    void chooseIono(bool in_flight, BuiltEpoch& out);
    void offerLite(const TimedRawEpoch& e, const ShadowNav& nav, const double r[3], BuiltEpoch& out);
    void capEpoch(const TimedRawEpoch& e);
    void capBuilt(BuiltEpoch& out);
    void floorKeep(BuiltEpoch& out) const;
    int64_t now() const { return clock_ ? clock_() : 0; }

    Config           cfg_;
    Clock            clock_ = nullptr;
    EphemerisStore   store_;
    RawMeasBuilder   builder_;
    const TimedRawEpoch* e_ = nullptr;   // the epoch being built
    int64_t          t0_us_ = 0;
    uint16_t         week_ = 0;
    uint16_t         last_week_ = 0;     // the last epoch's own week (0: it named none)
    double           last_tow_s_ = -1.0; // the last epoch's time of week (any epoch)
    ShadowIono       iono_in_ = {};
    bool             have_iono_ = false;
    bool             iono_dirty_ = true; // the builder's model is to be chosen again
    bool             use_broadcast_ = false;
    ShadowMeasStats  st_ = {};
    // the cap (sat_cap): the epoch of its picks, being built; the satellites
    // built from the last epoch's picks, and the picks a build dropped under
    // the elevation mask, a bit a PRN by system; the time of week the first
    // of those was dropped
    TimedRawEpoch    sel_;
    int              cap_ = 0;           // the cap in use: cfg_.sat_cap, or the governor's (useNavCap)
    uint64_t         picked_[3] = {};
    uint64_t         floor_picked_[3] = {};  // of picked_ before the build, the first cfg_.sat_cap (cap_ above it)
    uint64_t         masked_[3] = {};
    double           masked_tow_ = 0.0;
    // the cheaper build: each observation's ephemeris as see() looked it up
    // for the epoch offer() builds (by e's observations; by the cap's picks)
    const gps_eph_t* eph_of_[RawEpoch::kMaxObs] = {};
    const gps_eph_t* sel_eph_[RawEpoch::kMaxObs] = {};
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_MEAS_STAGE_H
