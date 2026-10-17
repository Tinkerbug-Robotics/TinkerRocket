// raw_nav_policy.h — the mosaic-only image's rules for its one navigation
// filter: when it is not started, when the barometer is locked out at speed,
// when the receiver is asked to send its navigation blocks again, when a
// filter that stopped in flight starts again, and when a restored flight's
// start waits for the barometer (pure, header-only, host-tested)
//
// Mosaic-only build (TR_RAW_NAV_MOSAIC: the V8 or V9 with a mosaic-G5, where
// the raw-GNSS filter is the only navigation filter and GpsInsEKF is not built
// in). main.cpp keeps the state and the clocks; these decide. The Beetle's
// image does not include this file.
//
// Tested in tests_cpp/test_raw_nav_policy.cpp.
#ifndef RAW_NAV_POLICY_H
#define RAW_NAV_POLICY_H

#include <cstdint>

namespace RawNavPolicy
{

// ---------------------------------------------------------------- crash loop
// A filter that crashes the flight computer on every boot would keep it out of
// its loop for good: no flight logic, no OTA to replace the image. So the boot
// after two crash resets in a row (a panic, a watchdog, the setup watchdog's
// restart: FcRescuePolicy's Abnormal) does not start the filter: it flies the
// barometer and the receiver's PVT (the voters, the main-deploy backstop) and
// reaches its loop, where it can take an update. The third boot of a crash
// loop, whatever the first boot ran: on the bench the power-on's boot may
// crash within its first minute, in flight the pad's boot has always run for
// minutes, and either way two crashes leave the third boot without the
// filter, after two reboot outages at most.
//
// The count is its own, not FcRescuePolicy's unproven_boots: that one counts
// the boot a power-on starts too, and a minute of loop clears it to 0, so the
// same two crashes read 3 after a power-on's short boot and 2 after the pad's
// long one. Here only the crash resets count -- the one that ends a boot that
// ran its minute too (the pad's, in flight). A minute of loop since clears
// it, and so does any fresh start (a power cycle always brings the filter
// back) or another image (an update, a rollback). Rescue mode
// (kMaxUnprovenBoots) is separate and needs the OC to hold no flight; this
// rule does not.
//
// RTC_NOINIT in main.cpp, as FcRescuePolicy::Record is: a cold boot leaves
// garbage there, which a fresh start never reads and crashValid() never trusts.
struct CrashRecord
{
    uint32_t magic;
    uint32_t image_id;   // FcRescuePolicy's: the partition the count belongs to
    uint8_t  crashes;    // crash resets in a row since a fresh start or a minute of loop
    uint8_t  reserved[3];
    uint32_t check;
};

inline constexpr uint32_t kCrashMagic      = 0x524E434Cu;   // "RNCL"
inline constexpr uint8_t  kCrashLoopResets = 2;             // the boot after this many does not start the filter

inline uint32_t crashCheckOf(const CrashRecord& r)
{
    return ~(r.magic ^ r.image_id ^ ((uint32_t)r.crashes * 0x01000193u));
}

inline CrashRecord crashSealed(CrashRecord r)
{
    r.magic = kCrashMagic;
    r.check = crashCheckOf(r);
    return r;
}

inline bool crashValid(const CrashRecord& r)
{
    return r.magic == kCrashMagic && r.check == crashCheckOf(r);
}

// At boot, before the filter starts: this boot's count. A crash reset adds
// one to the stored count of the same image; anything else starts over.
inline CrashRecord crashAtBoot(const CrashRecord& stored, bool reset_abnormal, uint32_t image_id)
{
    CrashRecord r{};
    r.image_id = image_id;
    if (reset_abnormal && crashValid(stored) && stored.image_id == image_id)
        r.crashes = stored.crashes < 0xFFu ? (uint8_t)(stored.crashes + 1u) : stored.crashes;
    return crashSealed(r);
}

// The loop ran FcRescuePolicy::kProvenRunMs: the crashes before were not a loop.
inline CrashRecord crashOnProven(CrashRecord r)
{
    r.crashes = 0;
    return crashSealed(r);
}

inline bool crashLoop(const CrashRecord& r)
{
    return crashValid(r) && r.crashes >= kCrashLoopResets;
}

// ---------------------------------------------------------- transonic lockout
// The barometer's transonic lockout reads the navigation filter's speed: over
// on_mps the barometer is out of the barometer vote, out of GnssAscentGate's
// judgement, out of the raw vote guard's and out of the filter itself, until
// the speed is under off_mps. On this image that speed is the raw filter's,
// which fuses the same receiver as the GPS voter: a filter that falsely read
// over 260 m/s near apogee would silence the one independent witness that
// could catch it (raw_vote_guard.h says it does not cover this). Two rules
// close that:
//
//   * A filter that has not run this boot has no speed of its own: its
//     velocity is where it will start. With the filter skipped (the
//     crash-loop rule) or not started, never anything else: unlocked. After a
//     restore it is the snapshot's, seconds old, and the snapshot is all the
//     reboot knows: a restore whose snapshot read over on_mps starts locked
//     (restored_at_speed, main.cpp's restore) and stays locked until the
//     restored filter runs and reads under off_mps -- or, if it never runs,
//     restore_lock_max_ms after the restore. A reboot in a transonic coast
//     keeps the barometer out as GpsInsEKF's image did, and the restore does
//     not take its start height from that barometer (restoreSeedStep).
//   * The lock can only begin on the filter's own speed from launch until
//     engage_after_burnout_ms after the burnout: a rocket's speed rises past
//     260 m/s only under thrust, so a real lockout always begins there, and a
//     coast or a descent is never locked out afresh. The second covers the
//     burnout detector's 50 ms and a tail-off. Once on, it holds until the
//     speed is under off_mps, as before: a real transonic coast keeps it, and
//     the filter's speed falls with the rocket's.
//   * Only a healthy filter's speed begins or holds it. A filter that stopped
//     in flight leaves its last good state held, carried forward on the IMU
//     alone and not healthy (RawNavAdapter::setHoldLastGood), and its restart
//     takes well under a second; a lock still on unhealthy_release_ms after
//     the filter last read healthy is let go, as a stopped filter on the
//     Beetle's adapter let it go at once (its velocity then read the start's).
//     The barometer backstop (Layer 2) never read the lockout, so letting it
//     go gives up no protection there.
//
// Not in flight (the pad, a sim's reset, after landing) it is off. A barometer
// test (its climb rate agreeing) was weighed and not taken: the static port
// reads low by its dynamic-pressure error, growing fastest under thrust
// (#1579's 0.0075 m/Pa against a few hundred m/s^2 of boost), so the
// barometer's rate can read near zero or falling just as a real lockout must
// begin.
//
// What it leaves: a filter that is already wrong and over 260 m/s while the
// motor burns locks the barometer out until its speed falls under 240 m/s;
// so does a snapshot that was.
struct MachConfig
{
    float    on_mps                  = 260.0f;   // config::BARO_MACH_LOCKOUT_ON
    float    off_mps                 = 240.0f;   // config::BARO_MACH_LOCKOUT_OFF
    uint32_t engage_after_burnout_ms = 1000;
    uint32_t unhealthy_release_ms    = 1000;     // a lock outlives the filter's health this long
    uint32_t restore_lock_max_ms     = 5000;     // a restore's lock, if the filter never runs
};

struct MachInputs
{
    bool     filter_has_run    = false;  // RawNavAdapter::hasRun()
    bool     filter_healthy    = false;  // RawNavAdapter::isHealthy()
    uint32_t unhealthy_ms      = 0;      // since it last read healthy (0 while it does, or before it runs)
    float    speed_mps         = 0.0f;   // |the filter's velocity| (getVelEst)
    bool     in_flight         = false;  // rocket_state == INFLIGHT
    bool     burnout           = false;  // burnout_detected
    uint32_t since_burnout_ms  = 0;      // now_ms - burnout_time_ms (when burnout)
    bool     restored_at_speed = false;  // this boot restored a flight over on_mps, with a filter started
    uint32_t since_restore_ms  = 0;      // now_ms - the restore's time (when restored_at_speed)
};

// Whether a restore starts locked: the snapshot's speed, and a filter to
// release it (none on a crash-loop boot, or a start that failed).
inline bool restoreLocks(bool filter_started, float snapshot_speed_mps, const MachConfig& c = MachConfig{})
{
    return filter_started && snapshot_speed_mps > c.on_mps;
}

// The lockout after this pass, from the one before.
inline bool machLockout(bool locked, const MachInputs& in, const MachConfig& c = MachConfig{})
{
    if (!in.in_flight)
        return false;
    if (!in.filter_has_run)
        return in.restored_at_speed && in.since_restore_ms < c.restore_lock_max_ms;
    if (locked)
    {
        if (!in.filter_healthy && in.unhealthy_ms >= c.unhealthy_release_ms)
            return false;
        return !(in.speed_mps < c.off_mps);   // a non-finite speed holds it, as before
    }
    if (!in.filter_healthy)
        return false;
    const bool may_begin = !in.burnout || in.since_burnout_ms < c.engage_after_burnout_ms;
    return may_begin && in.speed_mps > c.on_mps;
}

// ------------------------------------------------- navigation blocks again
// The mosaic-G5 sends each navigation block once, when it first decodes it,
// and again on an esoc (the driver's requestGNSSNavResend). The driver asks
// after every link setup and at launch (main.cpp enterInflight: the flight
// log's copy). On the pad the GNSS task asks again while it tracks satellites
// it has no ephemeris for -- a block the queue dropped, or one refused before
// the receiver's week was known -- at most once a period; the loop asks once
// on PRELAUNCH entry and once landed_delay_ms after LANDED, so the log around
// the flight holds them all.
inline constexpr uint32_t kPadResendPeriodMs   = 30000;
inline constexpr uint32_t kLandedResendDelayMs = 5000;

struct PadResend
{
    bool     started = false;
    uint32_t last_ms = 0;   // the last ask (the first pass stands for the boot's link setup)
    uint32_t sent    = 0;
};

// The GNSS task, each pass. True when it asks now.
inline bool padResendDue(PadResend& st, uint32_t now_ms, bool on_pad, int tracked_no_eph,
                         uint32_t period_ms = kPadResendPeriodMs)
{
    if (!st.started)
    {
        st.started = true;
        st.last_ms = now_ms;
    }
    if (!on_pad || tracked_no_eph <= 0)
        return false;
    if ((uint32_t)(now_ms - st.last_ms) < period_ms)
        return false;
    st.last_ms = now_ms;
    st.sent++;
    return true;
}

struct ResendEdges
{
    bool     was_prelaunch  = false;
    bool     was_landed     = false;
    bool     landed_pending = false;
    uint32_t landed_ms      = 0;
};

// The loop, each pass. True when it asks now: on PRELAUNCH entry, and once
// landed_delay_ms into LANDED (not if it leaves LANDED first).
inline bool edgeResendDue(ResendEdges& st, bool prelaunch, bool landed, uint32_t now_ms,
                          uint32_t landed_delay_ms = kLandedResendDelayMs)
{
    bool due = prelaunch && !st.was_prelaunch;
    if (landed && !st.was_landed)
    {
        st.landed_pending = true;
        st.landed_ms = now_ms;
    }
    if (!landed)
        st.landed_pending = false;
    if (st.landed_pending && (uint32_t)(now_ms - st.landed_ms) >= landed_delay_ms)
    {
        st.landed_pending = false;
        due = true;
    }
    st.was_prelaunch = prelaunch;
    st.was_landed = landed;
    return due;
}

// ------------------------------------------------------- restart in flight
// A filter that stops in flight (a state gone non-finite, a gap too long to
// bridge) leaves the adapter holding its last good state, carried forward on
// the IMU and not healthy (RawNavAdapter::setHoldLastGood). The loop starts it
// again from that state through the restore path -- at most kMaxRestarts a
// flight: a filter that keeps stopping is left held (unhealthy, so it casts
// no vote) rather than restarted for ever. Once per stop: the restore is
// pending until the filter runs.
inline constexpr uint8_t kMaxRestarts = 3;

inline bool restartDue(bool in_flight, bool stopped_in_flight, bool restore_pending, uint8_t restarts,
                       uint8_t max_restarts = kMaxRestarts)
{
    return in_flight && stopped_in_flight && !restore_pending && restarts < max_restarts;
}

// ------------------------------------------------ a restored flight's start
// After a reboot in flight the snapshot's height is as old as the boot: at
// 100 m/s a 3 s boot is 300 m. And the filter freezes the barometer's offset
// at its first sample in flight against the height it starts from, so a stale
// start biases every barometer update for the rest of the flight. So the
// restore's seed waits (at most wait_ms from the restore) for a barometer
// that can vouch for itself, and starts at the height it gives now -- the
// pad's datum plus its height above the pad. Without one by then it starts
// from the snapshot's height, and the console says so. A barometer locked out
// at the restore's speed (machLockout's restored_at_speed) cannot vouch for a
// height at all: the start takes the snapshot's at once, with no wait.
inline constexpr uint32_t kRestoreBaroWaitMs = 3000;

struct RestoreHold
{
    bool     armed = false;   // a reboot's restore is waiting for its start
    uint32_t t0_ms = 0;       // when the restore was taken
};

enum class RestoreSeed : uint8_t
{
    Publish,         // nothing held: the seed as it is
    Wait,            // not yet: no seed this pass
    Rebase,          // start at the barometer's height, then publish
    SnapshotHeight,  // no barometer in time: publish the snapshot's height
    Locked,          // the barometer locked out at the restore's speed: the snapshot's height, at once
};

inline void armRestoreHold(RestoreHold& h, uint32_t now_ms)
{
    h.armed = true;
    h.t0_ms = now_ms;
}

inline RestoreSeed restoreSeedStep(RestoreHold& h, bool restore_pending, bool baro_ok, uint32_t now_ms,
                                   bool baro_locked = false, uint32_t wait_ms = kRestoreBaroWaitMs)
{
    if (!h.armed || !restore_pending)
    {
        h.armed = false;
        return RestoreSeed::Publish;
    }
    if (baro_locked)
    {
        h.armed = false;
        return RestoreSeed::Locked;
    }
    if (baro_ok)
    {
        h.armed = false;
        return RestoreSeed::Rebase;
    }
    if ((uint32_t)(now_ms - h.t0_ms) >= wait_ms)
    {
        h.armed = false;
        return RestoreSeed::SnapshotHeight;
    }
    return RestoreSeed::Wait;
}

}  // namespace RawNavPolicy

#endif  // RAW_NAV_POLICY_H
