#pragma once
//
//  SimReplay.h — a flight's sensors, played on the flight computer against the
//  receiver's GPS time while the COCOM rig plays the same flight's GNSS
//  signals: the rig replay (#1600).
//
//  The rig plays a flight's GNSS signals from a SignalSim file to the
//  receiver in a Faraday cage; the raw-GNSS filter, the physics checks on the
//  GNSS (#1508) and the flight filter only see a flight if the IMU, the
//  barometer and the magnetometer fly it too. A TR_SIM_REPLAY build carries
//  the flight's sensors (tools/sim_replay_blob.py writes them from the sensors
//  CSV a sim_flight_3d.py run writes beside its scenario) and the sensor sim
//  (TR_Sensor_Collector_Sim) plays them from boot in place of the real
//  sensors' values: the pad while the receiver has no clock, then each sample
//  at its own GPS time. The receiver's output -- its fixes, raw measurements,
//  ephemerides -- passes through as the rig made it.
//
//  The time: a receiver fix carries its GPS time and that instant on the MCU
//  clock (TimedFix, GnssTimeSync), which places any MCU instant on the
//  replay's timeline, s from ignition. The clock follows the fixes while the
//  replay is on the pad and freezes before ignition (ReplayClock::Config::
//  arm_by_s): from there the samples run on the MCU's crystal, as a real
//  IMU's do, and the receiver's fixes are placed by the time sync as they are
//  in flight. A clock that arrives after that instant never starts the flight
//  (one would begin mid-boost): the replay stays on the pad and says so.
//  The mosaic-G5's driver hands no timed fix; its timed raw epochs (the
//  receiver's GPS week and time of week, that instant on the MCU from the
//  1PPS line) set the clock instead, ten a second (ReplayClock::onEpoch).
//
//  Frames: the firmware's rocket frame, FLU (x nose, y left, z up; see
//  SimSensorModel.h), and NED at the pad.
//
//  Pure C++ (no ESP), host-tested in tests_cpp/test_sim_replay.cpp.
//

#include <cstddef>
#include <cstdint>

namespace sim_replay {

// The carried flight's header (tools/sim_replay_blob.py has the layout).
struct Header
{
    uint16_t version = 0;
    uint16_t rate_hz = 0;
    uint32_t samples = 0;
    double   t0_s = 0.0;            // the first sample, s from ignition
    double   ignition_gps_s = 0.0;  // ignition's GPS time, week * 604800 + time of week, s
    double   lat_deg = 0.0, lon_deg = 0.0;
    double   alt_m = 0.0;           // the pad, above the ellipsoid
    double   scen_start_s = 0.0;    // the rig file's first instant, s from ignition (minus its prologue)
    double   scen_end_s = 0.0;      // its last
    float    acc_lsb = 0.0f;        // m/s^2
    float    gyro_lsb = 0.0f;       // rad/s
    float    p_ref_pa = 0.0f;
    float    p_lsb_pa = 0.0f;
    float    pad_loop_s = 0.0f;     // the first this many seconds are the pad, looped before the clock
    float    land_loop_s = 0.0f;    // the last this many are landed, looped after the end
    float    decimal_year = 0.0f;   // for the magnetic model, as the flight computer works it out
    char     name[28] = {};
};

struct Sample
{
    float   f[3] = {};       // specific force, FLU, m/s^2
    float   w[3] = {};       // angular rate, FLU, rad/s
    float   q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // attitude, FLU body -> NED, scalar first
    float   p_pa = 0.0f;     // the static pressure the barometer reads
    uint8_t phase = 0;       // 0 pad, 1 rail, 2 boost, 3 coast, 4 drogue, 5 main, 6 landed
    bool    high_g = false;  // the high-g channel was in use
};

class SimReplay
{
public:
    static constexpr size_t kHeaderBytes = 128;
    static constexpr size_t kSampleBytes = 24;

    // Checks and indexes a carried flight; the bytes are not copied and must
    // outlive this. False, with why(), if they are not one.
    bool load(const uint8_t* blob, size_t len);
    bool loaded() const { return blob_ != nullptr; }
    const char* why() const { return why_; }
    const Header& header() const { return h_; }

    double t0() const { return h_.t0_s; }
    double tLast() const;   // the last sample's time, s from ignition

    // The flight at t, s from ignition: interpolated between the samples
    // either side (the attitude normalised). Before the first sample, the pad
    // loop at t; after the last, the landed loop.
    void at(double t_s, Sample& out) const;

    // The pad loop at a free-running time (s, any origin): the pad before the
    // receiver has a clock.
    void padAt(double any_s, Sample& out) const;

    // Sample i as carried.
    void raw(uint32_t i, Sample& out) const;

private:
    void interp(double idx, Sample& out) const;

    const uint8_t* blob_ = nullptr;
    Header         h_;
    const char*    why_ = "not loaded";
};

// The replay's clock: an MCU instant on the replay's timeline, from the
// receiver's fixes.
class ReplayClock
{
public:
    struct Config
    {
        double arm_by_s = -5.0;    // follow the fixes until this, s from ignition, then freeze; a first
                                   // clock after it never starts the flight
        double margin_s = 30.0;    // fixes this far outside the rig file's span are not the rig's
        double restart_s = 0.5;    // a step this large in the fixes' clock is a new run of the file
        // the freeze checked against the 1PPS edge (onTagCheck): by a second
        // from the freeze's own on and no later than this, s from ignition --
        // still the pad, where the replayed rocket stands still
        double check_by_s = -2.0;
        double check_bar_us = 200.0;   // the edge this far from the frozen clock moves it (the driver's bar)
    };

    enum State : uint8_t
    {
        kWaiting = 0,   // no fix of the rig's yet: the pad, looped
        kFollowing,     // on the pad, on the fixes' clock
        kFrozen,        // from arm_by_s on: the MCU's crystal
        kMissed,        // the first clock came after arm_by_s: the pad for good
    };

    // The freeze against the 1PPS edge (onTagCheck)
    enum Check : uint8_t
    {
        kNotChecked = 0,   // not frozen yet, or no check of a second after the freeze yet
        kKept,             // the edge agreed with the frozen clock within check_bar_us
        kMoved,            // it did not: the clock moved onto the edge
        kUnchecked,        // no check came by check_by_s: the freeze stands as the tags placed it
    };

    ReplayClock() = default;
    explicit ReplayClock(const Config& cfg) : cfg_(cfg) {}

    void reset(const Header& h);

    // A fix: its GPS time (week * 604800 + time of week, s) and that instant
    // on the MCU clock (us); either 0 = not known.
    void onFix(double gps_s, int64_t mcu_us);

    // A timed raw epoch, for a receiver whose driver hands no timed fix (the
    // mosaic-G5: its epochs are what the time sync places): its full GPS week,
    // its time of week (s), that instant on the MCU clock (us), and the
    // receiver's clock bias its tag took off that time of week (s,
    // TimedRawEpoch::tag_dtr_ms). A week or an instant of 0 is not known yet
    // -- the epoch is not counted. The same clock as a fix's, at the
    // receiver's epoch rate (10 Hz on the mosaic). The time of week is the
    // receiver's own: the GPS instant mcu_us places is tow_s - dtr_s, and
    // that instant is the one the replay's time is set from -- so the clock
    // is the 1PPS line's own, and the replayed IMU sits on the sky's GPS time
    // (not up to the 0.5 ms of the bias off it), whatever the tag did.
    void onEpoch(uint16_t gps_week, double tow_s, int64_t mcu_us, double dtr_s = 0.0);

    // The mosaic-G5 driver's 1PPS check of a whole second's tag
    // (TR_GNSSReceiverMosaicSerial::RawStats::pps: the second, week * 604800 +
    // the receiver's time of week, s, and its residual, ppsResidualUs): the
    // tag of that second's epoch sat residual_us off where its 1PPS edge puts
    // it (about 0.03 ms when right). The tags come from the driver's clock
    // line, which follows a sky that came late -- a transmitter underrun --
    // only after about a minute (GnssTimeSync's envelope: the third-lowest of
    // the last 64 seconds); the edges follow it at once. A freeze inside that
    // minute would hold the replayed IMU that far off the sky for the whole
    // flight, consistent with the tags while they stay wrong, so the filter
    // would not see it either. So the first check of a second from the
    // freeze's own on (by check_by_s) confirms the freeze: where the edge puts
    // that second more than check_bar_us from where the frozen clock does,
    // the clock moves onto the edge -- the sky's own time, which the receiver
    // tracks -- on the pad, where the replayed rocket stands still. Never
    // after check_by_s. A second whose epoch was not counted is not paired.
    void onTagCheck(double sec_gps_s, int32_t residual_us);

    // The replay's time (s from ignition) at an MCU instant; false until a fix
    // has set the clock (or when it came too late).
    bool tAt(int64_t mcu_us, double& t_s) const;

    State state() const { return state_; }
    uint32_t fixes() const { return fixes_; }       // fixes that were the rig's
    uint32_t outside() const { return outside_; }   // fixes outside its span (another time, a stale clock)
    uint32_t restarts() const { return restarts_; }
    double lastFixT() const { return last_fix_t_; } // the latest rig fix, s from ignition
    // The freeze's check (onTagCheck): the state, the checked second (s from
    // ignition), its residual (us) and the move it made (us, edge less frozen)
    Check check() const { return check_; }
    double checkT() const { return check_t_; }
    int32_t checkResidualUs() const { return check_res_us_; }
    double checkMoveUs() const { return check_move_us_; }
    uint32_t tagChecks() const { return tag_checks_; }   // checks paired with a counted second

private:
    void noteCheckDeadline(double t);

    Config  cfg_;
    double  ignition_gps_s_ = 0.0;
    double  lo_s_ = 0.0, hi_s_ = 0.0;
    double  base_s_ = 0.0;          // replay time at MCU 0
    State   state_ = kWaiting;
    uint32_t fixes_ = 0, outside_ = 0, restarts_ = 0;
    double  last_fix_t_ = 0.0;
    double  freeze_t_ = 0.0;        // the freezing fix's time, s from ignition
    // the last two counted whole seconds' epochs, for onTagCheck: the
    // receiver's second, the epoch's time from ignition and the base its tag gave
    struct Second
    {
        int64_t sec = -1;
        double  t = 0.0;
        double  base = 0.0;
    };
    Second  secs_[2];
    uint8_t sec_next_ = 0;
    Check   check_ = kNotChecked;
    double  check_t_ = 0.0;
    int32_t check_res_us_ = 0;
    double  check_move_us_ = 0.0;
    uint32_t tag_checks_ = 0;
};

}  // namespace sim_replay
