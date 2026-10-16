// ShadowLink.h — what the flight loop (core 1) hands the raw-GNSS shadow
// filter (core 0) (#1600): every flight-filter IMU tick, each barometer
// sample the flight filter fuses, now and then the flight filter's own state
// to start from and to take its pad attitude from, and the ionosphere a pad
// calibration brought. And between the filter's task and the GNSS task that
// builds its epochs (RawGnssMeasStage.h, #1626), both on core 0: the
// ephemerides the receiver sent, the satellites the filter found jumped, and
// the GNSS task's counts (its built epochs go through a ring of their own,
// in PSRAM: main.cpp).
//
// The flight loop must not wait on core 0, so neither handoff can block it:
//   - IMU ticks go through a single-producer single-consumer ring, a store
//     and an index publish (a full ring drops the tick and counts it);
//   - the state goes through a seqlock, which the writer never waits on and
//     the reader retries if it caught a write half done.
// Pure C++ (std::atomic), host-tested in tests_cpp/test_raw_gnss_shadow.cpp.
#ifndef RAW_GNSS_SHADOW_LINK_H
#define RAW_GNSS_SHADOW_LINK_H

#include <atomic>
#include <cstdint>
#include <cstring>

namespace gnss_raw
{

// One flight-filter IMU tick: exactly the input GpsInsEKF::update was given.
struct ShadowImu
{
    uint32_t time_us;   // its time (esp_timer us, truncated to 32 bits)
    float    acc[3];    // specific force, body FRD, m/s^2
    float    gyro[3];   // body FRD, rad/s (the flight filter's input is deg/s)
    uint8_t  flags;     // kShadow* below
};
static constexpr uint8_t kShadowLevel    = 0x01;  // the flight filter levels against gravity on this tick
static constexpr uint8_t kShadowInFlight = 0x02;  // rocket_state is INFLIGHT
static constexpr uint8_t kShadowSim      = 0x04;  // a sensor sim runs: the IMU is not real
static constexpr uint8_t kShadowLanded   = 0x08;  // the landing detector has called it: zero velocity
static constexpr uint8_t kShadowApogee   = 0x10;  // the flight computer has called apogee (pyro_apogee_detected:
                                                  //   latched for the flight, restored after a reboot)

// A magnetometer sample (body FRD, uT): the flight filter's input. The raw
// filter takes it as a heading on the ground where it is the navigation
// filter (RawGnssShadow::Config::primary) -- and only if its hard iron is out
// (flags): an uncalibrated Beetle carries ~40 uT of it, and at the attitudes
// where that leaves |B| near the model's size it turns the heading by tens
// of degrees (#1590). Each sample says so itself, because a calibration can
// be applied while the filter runs.
struct ShadowMag
{
    uint32_t time_us;
    float    mag[3];
    uint8_t  flags;     // kShadowMag* below; 0: its hard iron is not known to be out
};
static constexpr uint8_t kShadowMagCalibrated = 0x01;  // a stored hard-iron calibration is applied
static constexpr uint8_t kShadowMagSim        = 0x02;  // a sensor sim's or the rig replay's field: the truth's

// A barometer sample the flight filter fused (the same gate: in range, not a
// spike, not a sealed port): its height above the pad from the ground
// pressure, m.
struct ShadowBaro
{
    uint32_t time_us;   // its time (esp_timer us, truncated to 32 bits)
    float    height_m;  // above the pad (pressure altitude from the ground pressure)
};

// The flight filter's state.
struct ShadowSeed
{
    uint32_t time_us;       // the IMU tick it is at
    double   lla[3];        // rad, rad, m (ellipsoidal)
    float    vel_ned[3];    // m/s
    float    quat[4];       // scalar-first, as both filters keep it
    float    gyro_bias[3];  // rad/s
    uint8_t  on_pad;        // READY or PRELAUNCH: still, and its heading is the magnetometer's
    uint8_t  pad_valid;     // the launch site below is set (the pad survey's reference, #1600 phase 5)
    float    pad_sigma_m;   // its uncertainty, m
    double   pad_lla[3];    // the launch site: the flight computer's average of the receiver's fix
                            // (rad, rad, m ellipsoidal)
    // where the raw filter is the navigation filter (Config::primary)
    float    declination_rad;  // true north east of magnetic, at the pad (TR_GeoMag)
    float    field_ut;         // the field's size the model gives there; 0 = no model
    float    dip_rad;          // and its dip, below the horizontal (#1590)
    uint8_t  have_declination;
    uint8_t  have_dip;         // dip_rad is the model's
    uint8_t  restore;          // a flight restored after a reboot: start in flight from this state
    uint8_t  start_gen;        // a new one on the pad (the flight computer started over): start again
};

// The raw filter's navigation state, every step (core 0 -> core 1): what the
// flight loop flies on where it is the navigation filter.
struct ShadowNav
{
    uint32_t time_us;          // the IMU tick it is at
    double   lla[3];           // rad, rad, m (ellipsoidal)
    float    vel_ned[3];       // m/s
    float    quat[4];          // body FRD -> NED, scalar first (as GpsInsEKF)
    float    accel_bias[3];    // m/s^2
    float    gyro_bias[3];     // rad/s
    float    var[15];          // its covariance's diagonal: position (m^2), velocity, attitude
                               // (half-angle, as GpsInsEKF's), accelerometer and gyro bias
    uint8_t  running;
    uint8_t  gnss_aided;       // raw measurements applied within the last 2 s
    uint8_t  baro_aided;       // a barometer sample fused within the last 1 s
    uint8_t  in_flight;        // its last IMU tick was INFLIGHT (the GNSS task keeps its ionosphere then)
    uint32_t epochs;           // raw epochs applied since it started
};

// The GPS broadcast ionosphere the app uploaded with a pad calibration
// (#1600 phase 5), as the flight computer keeps it in NVS.
struct ShadowIono
{
    int8_t   raw[8];        // alpha0..3, beta0..3 as GPS broadcasts them (RawMeasBuilder::ionoFromBroadcast)
    uint16_t gps_day;       // the day of the navigation file they came from: days since 1980-01-06
};

// A broadcast ephemeris as the receiver sent it (an RTCM3 payload): the GNSS
// task decodes it, and hands it on to the filter's for the pad context
// (EphemerisFrames).
struct ShadowEphFrame
{
    uint16_t len;
    uint8_t  data[72];         // EphemerisFrames::kMaxPayload, the driver's kEphMaxPayload
};

// A satellite whose pseudorange the filter's check found jumped (SatJumpCheck):
// relocked at that epoch, so the GNSS task's builder widens it from then on.
struct ShadowRelock
{
    uint8_t  sys;
    uint8_t  prn;
    double   tow_s;
};

// The GNSS task's side, each epoch it sees (RawGnssMeasStage::stats): the
// console's and the pad status's view of the receiver.
struct ShadowMeasStats
{
    uint32_t epochs;           // raw epochs seen
    uint32_t untimed;          //   with no MCU time yet
    uint32_t idle;             //   while the filter was not running
    uint32_t built;            //   built and handed on
    uint32_t dropped;          //   built, but the handoff was full
    uint32_t eph_added, eph_bad;
    int16_t  eph_sats;         // GPS, Galileo and BeiDou satellites with an ephemeris
    int16_t  tracked;          // the last epoch: those satellites with a pseudorange
    int16_t  tracked_no_eph;   //   and of them, with no ephemeris yet
    uint8_t  last_timed;       // the last epoch had its MCU time
    uint8_t  iono_age_known;   // the broadcast ionosphere's age by the receiver's date
    int16_t  iono_age_days;
    uint32_t build_us;         // the last epoch's build, wall time (preempted by everything above it)
    uint32_t build_us_max;
};

// Lock-free ring, one producer and one consumer on any cores.
template <typename T, uint32_t N>
class SpscRing
{
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

public:
    bool push(const T& v)
    {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= N)
        {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);   // the slot is written before it is published
        return true;
    }

    bool pop(T& v)
    {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);
        if (h == t)
            return false;
        v = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);   // the slot is read before it is freed
        return true;
    }

    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> dropped_{0};
    T buf_[N];
};

// A ring of N large items held elsewhere -- PSRAM on the flight computer,
// where the GNSS task's built epochs go (~6 kB each) -- one producer and one
// consumer: the indices here (internal RAM: atomics stay off PSRAM), the
// items there (attach). The producer fills a slot in place (claim, then
// publish), the consumer reads it in place (peek, then release).
template <typename T, uint32_t N>
class SpscSlots
{
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

public:
    void attach(T* items) { buf_ = items; }

    // The producer's next slot, or nullptr when all are waiting (or none attached).
    T* claim()
    {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);
        if (buf_ == nullptr || h - t >= N)
            return nullptr;
        return &buf_[h & (N - 1)];
    }

    // The claimed slot, filled: the consumer's to read.
    void publish()
    {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        head_.store(h + 1, std::memory_order_release);   // written before it is published
    }

    const T* peek() const
    {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);
        return (buf_ == nullptr || h == t) ? nullptr : &buf_[t & (N - 1)];
    }

    void release()
    {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        tail_.store(t + 1, std::memory_order_release);   // read before it is freed
    }

private:
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    T* buf_ = nullptr;
};

// One value, many writes, many reads; the writer never waits.
template <typename T>
class SeqLock
{
public:
    void write(const T& v)
    {
        const uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);         // odd: a write is under way
        std::atomic_thread_fence(std::memory_order_release);
        std::memcpy(raw_, &v, sizeof(T));
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 2, std::memory_order_relaxed);         // even again: done
    }

    // False before the first write, or if every try met a write under way.
    // `seq` tells one write from the next.
    bool read(T& out, uint32_t& seq) const
    {
        for (int tries = 0; tries < 8; ++tries)
        {
            const uint32_t s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1u)
                continue;
            std::memcpy(&out, raw_, sizeof(T));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s1)
            {
                seq = s1;
                return s1 != 0;
            }
        }
        return false;
    }

private:
    std::atomic<uint32_t> seq_{0};
    alignas(8) unsigned char raw_[sizeof(T)] = {};
};

// What the shadow tells the pad calibration's status message (#1600 phase 5),
// once a second from its task (RawGnssShadow::padStatus).
struct ShadowPadStatus
{
    uint8_t running;          // the filter is running
    uint8_t tracked;          // GPS, Galileo, BeiDou satellites with a pseudorange, last epoch
    uint8_t no_eph;           //   of those, no ephemeris yet
    uint8_t settling;         //   pad survey: locked too briefly to survey
    uint8_t corrected;        //   pad survey: a correction ready
    uint8_t iono_source;      // RawGnssShadow::IonoSource (= PAD_IONO_*)
    uint8_t iono_age_known;
    int16_t iono_age_days;    // the broadcast parameters' age by the receiver's date
    float   corr_rms_m;       // the corrections' spread
    float   pos_sigma_m;      // the filter's 3-D position sigma
    uint8_t pad_mismatch;     // the raw measurements put the pad far from the receiver's average
    float   pad_offset_m;     //   this far
    uint8_t fix_suspect;      // the receiver's last fix moved as the IMU says the rocket did not
};

// The handoffs, as one object both sides share.
struct ShadowLink
{
    // Room for the raw filter's longest pause. An epoch built and fused in one
    // pass held it about 5 ms a satellite on the Beetle (r3: 14 satellites,
    // 67 ms; up to 97 ms); now the GNSS task builds it and the filter fuses it
    // a slice a step, but the margin stays. The navigation filter is fed about
    // 450 samples a second (the shadow 171): 0.57 s here.
    SpscRing<ShadowImu, 256> imu;
    SpscRing<ShadowBaro, 64> baro;
    SpscRing<ShadowMag, 64> mag;   // where it is the navigation filter
    SeqLock<ShadowNav> nav;        // core 0 -> core 1 (and the GNSS task), every step
    SeqLock<ShadowSeed> seed;
    SeqLock<ShadowIono> iono;      // written at boot (from NVS) and by each pad calibration
    SeqLock<ShadowPadStatus> pad;  // core 0 -> core 1, once a second
    // between the filter's task and the GNSS task (#1626)
    SpscRing<ShadowEphFrame, 8> eph;      // GNSS task -> filter's: each ephemeris, for the pad context
    SpscRing<ShadowRelock, 16> relock;    // filter's -> GNSS task: each satellite found jumped
    SeqLock<ShadowMeasStats> meas;        // GNSS task -> filter's: its counts, each epoch
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_SHADOW_LINK_H
