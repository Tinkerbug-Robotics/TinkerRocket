// EphemerisStore.h — the broadcast ephemerides a raw-GNSS filter places satellites by
//
// Filled from the receiver's RTCM ephemeris (RtcmEphemeris.h) as it arrives,
// a satellite every second or so; read once per satellite per raw epoch. A
// fixed pool rather than a table indexed by system and PRN: the LC86G tracks
// about 30 satellites at a time, and a table for every GPS, Galileo and
// BeiDou PRN would hold ~130 ephemerides of ~260 bytes for ~30 in use.
//
// Each satellite keeps its two latest ephemerides (by toe), so the one in use
// is still there for the epochs that straddle an update. Pure and
// host-compilable (tests_cpp/test_gnss_raw.cpp).
#ifndef EPHEMERIS_STORE_H
#define EPHEMERIS_STORE_H

#include <cstddef>

#include "gnss/eph.h"

namespace gnss_raw
{

class EphemerisStore
{
public:
    static constexpr size_t kCapacity = 64;   // two each for 32 satellites
    static constexpr int    kPerSat   = 2;

    EphemerisStore() { clear(); }
    void clear();

    // File a decoded ephemeris. A repeat of one held (same IODE and toe) is
    // ignored; a third for a satellite replaces its oldest; with the pool full,
    // the ephemeris whose toe is furthest from this one's goes. True if it was new.
    bool add(const gps_eph_t& e);

    // The ephemeris to place sys/prn by at time t (s of week in that system's
    // own time, as gnss/eph.h takes it): the one with toe nearest t, if within
    // kMaxAgeS of it and healthy; nullptr otherwise. Nearest within 2 h is the
    // bench prototype's rule (gnss/sim/tinkerrocket_gnss/gnss_raw.py, pick).
    const gps_eph_t* select(int sys, int prn, double t) const;

    // Satellites of one system (gnss_sys_t) with at least one ephemeris held.
    int satellites(int sys) const;
    size_t size() const { return used_; }

    static constexpr double kMaxAgeS = 7200.0;

private:
    gps_eph_t pool_[kCapacity];
    size_t    used_ = 0;
};

}  // namespace gnss_raw

#endif  // EPHEMERIS_STORE_H
