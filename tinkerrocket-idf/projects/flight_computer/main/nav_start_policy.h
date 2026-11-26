#pragma once

#include <math.h>
#include <stdint.h>

#include <RocketComputerTypes.h>   // RocketState

// When the navigation filter starts without GNSS, on every receiver (#1762).
//
// The filter starts normally on a fix that passes the init gate with the pad
// datum. Without one it has the degraded start: the IMU and the barometer from
// a pad-relative 0 N 0 E, GNSS held out (#557). That start used to be
// GNSS-absent mode alone: a receiver that failed its bring-up, and only while
// no fix had ever come (!gnss_started). A receiver that answers but never
// gives a usable fix -- a blocked sky, an antenna lead off, a carrier that
// tracks nothing, a 3-D fix held at 4 satellites (PRELAUNCH takes 4, the start
// 5), a fix lost before a pad re-init -- left the filter unstarted for the
// flight: no attitude, velocity or position, guidance off, and no transonic
// barometer lockout (it is computed only in the filter's update), so the
// barometer stayed an apogee voter and its backstop armed through Mach 1.
// The gnss-sdr-dev board's image had already met this (sdr_nav_start_policy.h,
// whose rule moved here and now covers every receiver):
//   - a receiver that failed its bring-up starts degraded at once (after the
//     gyro's average and the settle), as GNSS-absent mode always has;
//   - any other starts degraded once no usable fix has come kNoFixMs after
//     boot on the pad (READY), and at once in PRELAUNCH, where launch may come
//     any time -- never in flight, where the accelerometer is no measure of
//     the tilt;
//   - "no usable fix" means none for a while, not none on this loop pass
//     (#1756): the degraded start waits while a usable fix came in the last
//     kUsableFixMs (the next one starts the filter normally), and for
//     kReinitGraceMs after a re-init of a filter that had started on fixes
//     (a mag-cal exit, an orientation re-snap or setting change) --
//     receiverWait();
//   - a usable fix that comes later, while the rocket is still on the pad
//     (READY or PRELAUNCH) and has been at rest for kRestartStillMs
//     (RestWatch: the start takes its attitude from the accelerometer and its
//     gyro bias from the gyro, so not while the rocket is carried, and in
//     PRELAUNCH not once a lit motor's thrust shows), starts the filter over
//     on it (restartOnFix); once
//     it flies, the degraded start stands for the flight, and the receiver's
//     fixes stay out of the filter, its pad datum and pad average
//     (fixesHeldOut). On the pad they go on building the datum, which the
//     restart needs: PRELAUNCH is no arm. The FC enters it on its own, 3 s
//     after a 3-D fix with 4 satellites while the OC answers, and the start
//     needs 5, so a cold start whose first fixes have 4 enters PRELAUNCH,
//     starts degraded at once, and starts over on its first usable fix. (The
//     raw filter's epochs are held out while any degraded start stands.)
//   - a reboot in flight keeps a degraded start (snapshotFilterByte,
//     restoredWithoutGnss): the restored filter's position is the degraded
//     start's, no place for the receiver's fixes or epochs either.
// Pure, host-tested (tests_cpp/test_nav_start_policy.cpp).
namespace nav_start
{

static constexpr uint32_t kNoFixMs       = 120000u;   // no usable fix 2 min after boot: start without GNSS
static constexpr uint32_t kUsableFixMs   = 3000u;     // a usable fix this recent: the next one starts the filter
static constexpr uint32_t kReinitGraceMs = 5000u;     // after a re-init of a filter started on fixes
static constexpr uint32_t kRestartStillMs = 1000u;    // a restart waits until the rocket has been at rest this long
static constexpr double   kStandardGravityMps2 = 9.80665;

// The degraded start waits for the receiver (#1756): a usable fix (one the
// normal start would take: the init gate with the pad datum) came within
// kUsableFixMs, or a filter that had started on fixes was re-initialized
// within kReinitGraceMs. last_usable_ms / reinit_ms: millis() of each, 0 for
// never. Ages in unsigned arithmetic, across the wrap.
inline bool receiverWait(uint32_t last_usable_ms, uint32_t reinit_ms, uint32_t now_ms)
{
    if (last_usable_ms != 0 && (uint32_t)(now_ms - last_usable_ms) < kUsableFixMs)
        return true;
    return reinit_ms != 0 && (uint32_t)(now_ms - reinit_ms) < kReinitGraceMs;
}

// The degraded start is due: no usable fix to start on now and none to wait
// for (normal_ready: a normal start due this pass, or receiverWait()), the
// gyro's bias averaged (gyro_ready), the GNSS-absent settle over (settled:
// main.cpp's gnss_absent_dwell_ms, 5 s after boot and after a sim's reset, so
// a sim's first fix starts it normally), and then: at once for a receiver
// that failed its bring-up (receiver_absent: GNSS-absent mode), otherwise
// kNoFixMs since boot in READY, or at once in PRELAUNCH.
inline bool degradedStartDue(bool normal_ready, bool gyro_ready, bool settled, uint32_t since_boot_ms,
                             RocketState state, bool receiver_absent)
{
    if (normal_ready || !gyro_ready || !settled)
        return false;
    if (receiver_absent)
        return true;
    if (state == PRELAUNCH)
        return true;
    return state == READY && since_boot_ms >= kNoFixMs;
}

// The rocket is on the pad, waiting: READY, or PRELAUNCH (the FC's own step
// once the receiver and the OC are up, no arm).
inline bool onThePad(RocketState state)
{
    return state == READY || state == PRELAUNCH;
}

// The rocket at rest, for the restart: judged over kRestBlockMs blocks, not
// sample by sample, with the limits TR_Orientation's OrientationEstimator and
// InertialVertical judge a pad's rest by (#1755, whose per-sample test threw
// away a racked rocket in ordinary rail sway). A block passes when each gyro
// axis's mean is under kRestGyroMeanDps (a bias, not a turn) and its spread
// under kRestGyroRangeDps (rail sway passes), and the specific force's size
// spreads under kRestAccRangeMps2 with its mean within kRestAccTolMps2 of 1 g
// (handling and transport fail; an uncalibrated accelerometer's offset, at
// most ~1 m/s^2, passes; so does a bounce along gravity, which tilts nothing).
// A spread past its limit fails the block on that pass. At rest once
// kRestartStillMs of consecutive blocks passed. On every pass, a specific force more than kRestVetoMps2 from 1 g (a
// lit motor's thrust past ~1.25 times the weight -- launch counts readings
// over 2 g -- or a drop) ends the rest at once. (Thrust under the weight
// reads 1 g: the pad carries the rest, and the accelerometer's tilt is still
// the pad's.) Pure; host-tested.
static constexpr uint32_t kRestBlockMs      = 250u;
static constexpr double   kRestGyroMeanDps  = 5.0;
static constexpr double   kRestGyroRangeDps = 30.0;
static constexpr double   kRestAccRangeMps2 = 3.0;
static constexpr double   kRestAccTolMps2   = 2.0;
static constexpr double   kRestVetoMps2     = 2.5;

struct RestWatch
{
    uint32_t rest_ms = 0;        // consecutive passing blocks, ms
    uint32_t blk_start_ms = 0;
    uint32_t blk_n = 0;
    double   g_sum[3] = {0, 0, 0}, g_min[3] = {0, 0, 0}, g_max[3] = {0, 0, 0};
    double   a_sum = 0, a_min = 0, a_max = 0;

    void feed(uint32_t now_ms, double ax, double ay, double az, double gx, double gy, double gz)
    {
        const double a = sqrt(ax * ax + ay * ay + az * az);
        if (fabs(a - kStandardGravityMps2) > kRestVetoMps2)
        {
            rest_ms = 0;
            blk_n   = 0;
            return;
        }
        const double g[3] = {gx, gy, gz};
        if (blk_n == 0)
        {
            blk_start_ms = now_ms;
            a_sum = 0;
            a_min = a_max = a;
            for (int i = 0; i < 3; ++i) { g_sum[i] = 0; g_min[i] = g_max[i] = g[i]; }
        }
        ++blk_n;
        a_sum += a;
        if (a < a_min) a_min = a;
        if (a > a_max) a_max = a;
        for (int i = 0; i < 3; ++i)
        {
            g_sum[i] += g[i];
            if (g[i] < g_min[i]) g_min[i] = g[i];
            if (g[i] > g_max[i]) g_max[i] = g[i];
        }
        // a spread already past its limit fails the block now, not at its
        // end: the rocket picked up ends the rest within the motion's first
        // tens of ms
        bool spread_ok = a_max - a_min < kRestAccRangeMps2;
        for (int i = 0; i < 3; ++i)
            spread_ok = spread_ok && g_max[i] - g_min[i] < kRestGyroRangeDps;
        if (!spread_ok)
        {
            rest_ms = 0;
            blk_n   = 0;
            return;
        }
        const uint32_t len = now_ms - blk_start_ms;
        if (len < kRestBlockMs)
            return;
        bool pass = fabs(a_sum / blk_n - kStandardGravityMps2) < kRestAccTolMps2;
        for (int i = 0; i < 3; ++i)
            pass = pass && fabs(g_sum[i] / blk_n) < kRestGyroMeanDps;
        rest_ms = pass ? rest_ms + len : 0u;
        if (rest_ms > 0x7FFFFFFFu) rest_ms = 0x7FFFFFFFu;
        blk_n = 0;
    }

    bool atRest() const { return rest_ms >= kRestartStillMs; }
};

// A degraded start gives way to the receiver's first usable fix: only on the
// pad, and only once the rocket is at rest (still: RestWatch::atRest). A
// restart in a lit motor's thrust, before launch is called, would start the
// filter from a thrust-loaded attitude and on the raw-GNSS images miss the
// pad's restart of the raw filter.
inline bool restartOnFix(bool degraded_started, bool normal_ready, RocketState state, bool still)
{
    return degraded_started && normal_ready && onThePad(state) && still;
}

// A degraded start keeps the receiver's fixes out of the filter's pad datum
// and pad average once the rocket flies (INFLIGHT, LANDED); on the pad they
// build the datum the restart needs.
inline bool fixesHeldOut(bool degraded_started, RocketState state)
{
    return degraded_started && (state == INFLIGHT || state == LANDED);
}

// The snapshot's ekf_initialized byte (RocketComputerTypes.h SNAP_EKF_*): the
// filter started, and whether on a degraded start, so a reboot in flight
// keeps it. Every older writer wrote 0 or 1: started on fixes, as restored
// before.
inline uint8_t snapshotFilterByte(bool started, bool degraded_started)
{
    if (!started) return 0;
    return (uint8_t)(SNAP_EKF_STARTED | (degraded_started ? SNAP_EKF_WITHOUT_GNSS : 0u));
}

inline bool restoredWithoutGnss(uint8_t snap_byte)
{
    return (snap_byte & SNAP_EKF_STARTED) != 0 && (snap_byte & SNAP_EKF_WITHOUT_GNSS) != 0;
}

}  // namespace nav_start
