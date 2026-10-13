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
    };

    enum State : uint8_t
    {
        kWaiting = 0,   // no fix of the rig's yet: the pad, looped
        kFollowing,     // on the pad, on the fixes' clock
        kFrozen,        // from arm_by_s on: the MCU's crystal
        kMissed,        // the first clock came after arm_by_s: the pad for good
    };

    ReplayClock() = default;
    explicit ReplayClock(const Config& cfg) : cfg_(cfg) {}

    void reset(const Header& h);

    // A fix: its GPS time (week * 604800 + time of week, s) and that instant
    // on the MCU clock (us); either 0 = not known.
    void onFix(double gps_s, int64_t mcu_us);

    // The replay's time (s from ignition) at an MCU instant; false until a fix
    // has set the clock (or when it came too late).
    bool tAt(int64_t mcu_us, double& t_s) const;

    State state() const { return state_; }
    uint32_t fixes() const { return fixes_; }       // fixes that were the rig's
    uint32_t outside() const { return outside_; }   // fixes outside its span (another time, a stale clock)
    uint32_t restarts() const { return restarts_; }
    double lastFixT() const { return last_fix_t_; } // the latest rig fix, s from ignition

private:
    Config  cfg_;
    double  ignition_gps_s_ = 0.0;
    double  lo_s_ = 0.0, hi_s_ = 0.0;
    double  base_s_ = 0.0;          // replay time at MCU 0
    State   state_ = kWaiting;
    uint32_t fixes_ = 0, outside_ = 0, restarts_ = 0;
    double  last_fix_t_ = 0.0;
};

}  // namespace sim_replay
