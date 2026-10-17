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
// its loop for good: no flight logic, no OTA to replace the image. So once two
// boots in a row have ended in a crash reset (a panic or a watchdog) before
// the loop ran a minute, the third does not start the filter: it flies the
// barometer and the receiver's PVT (the voters, the main-deploy backstop) and
// reaches its loop, where it can take an update. FcRescuePolicy's count is the
// one used: atBoot() has counted this boot already, so the third boot of the
// run reads 3. A power-on starts the count over, so a power cycle always
// brings the filter back. Rescue mode itself (kMaxUnprovenBoots) is one boot
// further and needs the OC to hold no flight; this rule does not.
inline constexpr uint8_t kCrashLoopBoots = 3;   // this boot and the two crashed before it

inline bool crashLoop(bool reset_abnormal, uint8_t unproven_boots)
{
    return reset_abnormal && unproven_boots >= kCrashLoopBoots;
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
//   * A filter that has not run this boot has no speed: its velocity is where
//     it will start -- after a restore, the snapshot's, seconds old, or with
//     the filter skipped (the crash-loop rule) never anything else. Unlocked.
//   * The lock can only begin from launch until engage_after_burnout_ms after
//     the burnout: a rocket's speed rises past 260 m/s only under thrust, so a
//     real lockout always begins there, and a coast or a descent is never
//     locked out afresh. The second covers the burnout detector's 50 ms and a
//     tail-off. Once on, it holds until the speed is under off_mps, as before:
//     a real transonic coast keeps it, and the filter's speed falls with the
//     rocket's.
//
// Not in flight (the pad, a sim's reset, after landing) it is off. A barometer
// test (its climb rate agreeing) was weighed and not taken: the static port
// reads low by its dynamic-pressure error, growing fastest under thrust
// (#1579's 0.0075 m/Pa against a few hundred m/s^2 of boost), so the
// barometer's rate can read near zero or falling just as a real lockout must
// begin.
//
// What it leaves: a filter that is already wrong and over 260 m/s while the
// motor burns locks the barometer out until its speed falls under 240 m/s.
struct MachConfig
{
    float    on_mps                  = 260.0f;   // config::BARO_MACH_LOCKOUT_ON
    float    off_mps                 = 240.0f;   // config::BARO_MACH_LOCKOUT_OFF
    uint32_t engage_after_burnout_ms = 1000;
};

struct MachInputs
{
    bool     filter_has_run   = false;  // RawNavAdapter::hasRun()
    float    speed_mps        = 0.0f;   // |the filter's velocity| (getVelEst)
    bool     in_flight        = false;  // rocket_state == INFLIGHT
    bool     burnout          = false;  // burnout_detected
    uint32_t since_burnout_ms = 0;      // now_ms - burnout_time_ms (when burnout)
};

// The lockout after this pass, from the one before.
inline bool machLockout(bool locked, const MachInputs& in, const MachConfig& c = MachConfig{})
{
    if (!in.filter_has_run || !in.in_flight)
        return false;
    if (locked)
        return !(in.speed_mps < c.off_mps);   // a non-finite speed holds it, as before
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
// from the snapshot's height, and the console says so.
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
};

inline void armRestoreHold(RestoreHold& h, uint32_t now_ms)
{
    h.armed = true;
    h.t0_ms = now_ms;
}

inline RestoreSeed restoreSeedStep(RestoreHold& h, bool restore_pending, bool baro_ok, uint32_t now_ms,
                                   uint32_t wait_ms = kRestoreBaroWaitMs)
{
    if (!h.armed || !restore_pending)
    {
        h.armed = false;
        return RestoreSeed::Publish;
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
