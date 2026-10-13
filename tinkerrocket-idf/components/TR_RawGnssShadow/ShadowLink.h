// ShadowLink.h — what the flight loop (core 1) hands the raw-GNSS shadow
// filter (core 0) (#1600): every flight-filter IMU tick, now and then the
// flight filter's own state to start from and to take its pad attitude from,
// and the ionosphere a pad calibration brought.
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
};

// The GPS broadcast ionosphere the app uploaded with a pad calibration
// (#1600 phase 5), as the flight computer keeps it in NVS.
struct ShadowIono
{
    int8_t   raw[8];        // alpha0..3, beta0..3 as GPS broadcasts them (RawMeasBuilder::ionoFromBroadcast)
    uint16_t gps_day;       // the day of the navigation file they came from: days since 1980-01-06
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
};

// The handoffs, as one object both sides share.
struct ShadowLink
{
    SpscRing<ShadowImu, 64> imu;   // 0.37 s at the Beetle's 171 Hz flight filter
    SeqLock<ShadowSeed> seed;
    SeqLock<ShadowIono> iono;      // written at boot (from NVS) and by each pad calibration
    SeqLock<ShadowPadStatus> pad;  // core 0 -> core 1, once a second
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_SHADOW_LINK_H
