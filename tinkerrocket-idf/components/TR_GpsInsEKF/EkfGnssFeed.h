// EkfGnssFeed — the GNSS input the flight loop hands the EKF on every tick
// (issue #1107).  Pure header, no ESP-IDF dependencies, shared by the flight
// computer and the rocket-computer-mini, host-tested in tests_cpp.
//
// THE PROBLEM THIS SOLVES.  The flight loop polls the receiver far faster than
// it produces fixes (~480 EKF ticks/s against ~18 fixes/s), so the same fix
// sits in gnss_latest_si for ~25 ticks.  The EKF dedups on the fix time_us:
// updateCore() fuses a GNSS measurement only when its time_us differs from
// the last one fused (timeWeekPrev_), and skips it otherwise.  main.cpp used
// to keep a SECOND set of "consumed" markers, advanced whenever it called
// ekf.update() with a fresh fix, and once they said "seen" it handed the
// EKF a zero-initialised EkfGNSSDataLLA carrying the marker's timestamp —
// "pass a stale timestamp so the EKF skips".  That only works while the two
// bookkeepings agree.  They did not:
//
//   #367  the markers advanced on an EKF-off decimation tick, so the fix was
//         never fused, and the next EKF tick got zeros under a timestamp the
//         filter had never seen -> fused as a real fix.
//   #1107 the markers advanced on a tick where updateCore() returned at the
//         #440 frozen-IMU-timestamp skip, before its GNSS block.  Same
//         outcome: the next tick fused lat=0 / lon=0 / alt=0 / vel=0.
//         Measured on the host against the real filter: the descent-rate
//         estimate steps ~1.5 m/s toward zero per occurrence, and position
//         steps 0.06 m (converged) to 5 m (2 m position sigma) toward the
//         Gulf of Guinea; the fabricated zero velocity also poisons the
//         accel-match heading aiding on this fix and the next real one.
//         Flight logs show 0-7 such stall-on-fix coincidences per flight.
//
// THE RULE.  Consumption is tracked in exactly one place — the filter.  The
// loop holds the most recent ACCEPTED fix here and hands the EKF that struct,
// unchanged, on every tick until a newer fix replaces it.  A fix the filter
// did not get to (a frozen-dt skip, an EKF-off tick) is simply offered again
// on the next tick and fused then; a fix it already fused is a no-op.  There
// is no placeholder value, so there is nothing fabricated to fuse, whatever
// early return updateCore() grows next.  (The closed-loop simulation has
// always fed the EKF this way.)
//
// The quality gate (fix mode, satellites, h_acc) is NOT here: its thresholds
// are per-board config, so the caller evaluates it and passes the verdict.
#pragma once

#include <cstdint>
#include <cmath>

#include <RocketComputerTypes.h>   // GNSSDataSI — the converter's SI record
#include "TR_GpsInsEKF.h"          // EkfGNSSDataLLA

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class EkfGnssFeed
{
public:
    // Present the receiver's latest record; call once per flight-loop tick,
    // BEFORE deciding whether the EKF runs this tick.  `accepted` is the
    // caller's quality verdict on this record.
    //
    // Returns true exactly once per newly ARRIVED record that was accepted —
    // the once-per-fix work (GNSS noise scaling, launch-site averaging) runs
    // on true.  Arrival is keyed on the receiver's own fix time (second +
    // millisecond): the loop re-reads the same fix for ~25 ticks, and time_us
    // is the loop's receive stamp, which the EKF uses for its own dedup.
    // A rejected record is not held and does not advance the arrival key, so
    // it is re-evaluated (and rejected again) on every tick until replaced.
    bool offer(const GNSSDataSI& latest, bool accepted)
    {
        if (!accepted) return false;
        if (have_fix_ &&
            latest.second == last_second_ &&
            latest.milli_second == last_ms_) {
            return false;                       // the fix already held
        }
        static constexpr double DEG2RAD = M_PI / 180.0;
        held_.time_us   = latest.time_us;
        held_.lat_rad   = latest.lat * DEG2RAD;
        held_.lon_rad   = latest.lon * DEG2RAD;
        held_.alt_m     = latest.alt;
        held_.vel_n_mps = (float)latest.vel_n;
        held_.vel_e_mps = (float)latest.vel_e;
        held_.vel_d_mps = -(float)latest.vel_u; // ENU U -> NED D
        last_second_    = latest.second;
        last_ms_        = latest.milli_second;
        have_fix_       = true;
        return true;
    }

    // The GNSS argument for ekf.update() / ekf.init() this tick: the most
    // recent accepted fix, held unchanged until a newer one replaces it.
    // Before the first accepted fix it is all zeros with time_us 0, which a
    // filter initialised with time_us 0 (the GNSS-absent degraded init)
    // skips by the same equal-timestamp rule.
    const EkfGNSSDataLLA& current() const { return held_; }

    bool haveFix() const { return have_fix_; }

    // Forget the held fix (a sim start/stop re-arms the whole EKF path).  The
    // next accepted record is reported as new even if its fix time repeats.
    void reset()
    {
        held_       = EkfGNSSDataLLA{};
        have_fix_   = false;
        last_second_ = 0xFF;
        last_ms_     = 0xFFFF;
    }

private:
    EkfGNSSDataLLA held_ = {};
    bool     have_fix_    = false;
    uint8_t  last_second_ = 0xFF;
    uint16_t last_ms_     = 0xFFFF;
};
