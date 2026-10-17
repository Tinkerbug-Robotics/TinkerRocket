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
//
// The pool's size is a build's (TR_EPH_CAPACITY): 64 by default, the
// Beetle's. The mosaic-G5 build takes 128, set project-wide: that receiver
// sends the navigation data of every GPS, Galileo and BeiDou satellite it
// tracks, ~50 different ones over a 2-4 h pad wait, and two each of them is
// ~100 (tests_cpp/test_eph_store_pad_wait.cpp).
#ifndef EPHEMERIS_STORE_H
#define EPHEMERIS_STORE_H

#include <cstddef>

#include "gnss/eph.h"

namespace gnss_raw
{

class EphemerisStore
{
public:
    // No default #define of TR_EPH_CAPACITY: a build that does not set it
    // sees no macro it did not see before (tools/m1_identity.sh diffs them).
#ifdef TR_EPH_CAPACITY
    static constexpr size_t kCapacity = TR_EPH_CAPACITY;
#else
    static constexpr size_t kCapacity = 64;   // two each for 32 satellites
#endif
    static constexpr int    kPerSat   = 2;
    static_assert(kCapacity >= static_cast<size_t>(kPerSat), "TR_EPH_CAPACITY holds at least one satellite's");

#ifdef TR_EPH_CAPACITY
    // The kCapacity EphemerisStore.cpp was compiled with. Every translation
    // unit that sees this class lays the pool out by its own kCapacity, and
    // nothing at link time catches two of them, so a build that sets
    // TR_EPH_CAPACITY checks at boot that its components agree
    // (RawGnssMeasStage::ephCapacityAgrees). A component compiled without it
    // does not define this, and the image does not link.
    static size_t builtCapacity();
#endif

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
