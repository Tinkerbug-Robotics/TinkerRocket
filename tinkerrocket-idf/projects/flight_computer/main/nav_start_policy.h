#pragma once

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
//     boot on the pad (READY), and at once when the rocket is armed
//     (PRELAUNCH) -- never in flight, where the accelerometer is no measure
//     of the tilt;
//   - "no usable fix" means none for a while, not none on this loop pass
//     (#1756): the degraded start waits while a usable fix came in the last
//     kUsableFixMs (the next one starts the filter normally), and for
//     kReinitGraceMs after a re-init of a filter that had started on fixes
//     (a mag-cal exit, an orientation re-snap or setting change) --
//     receiverWait();
//   - a usable fix that comes later, while the rocket is still READY (on the
//     pad, unarmed), starts the filter over on it (restartOnFix); once armed
//     or flying, the degraded start stands for the flight, and the
//     receiver's fixes and raw epochs stay out of the filter and its pad
//     datum (its position is no place to build them at).
// Pure, host-tested (tests_cpp/test_nav_start_policy.cpp).
namespace nav_start
{

static constexpr uint32_t kNoFixMs       = 120000u;   // no usable fix 2 min after boot: start without GNSS
static constexpr uint32_t kUsableFixMs   = 3000u;     // a usable fix this recent: the next one starts the filter
static constexpr uint32_t kReinitGraceMs = 5000u;     // after a re-init of a filter started on fixes

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
// kNoFixMs since boot on the pad or the rocket armed.
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

// A degraded start gives way to the receiver's first usable fix: only on the
// pad, unarmed.
inline bool restartOnFix(bool degraded_started, bool normal_ready, RocketState state)
{
    return degraded_started && normal_ready && state == READY;
}

}  // namespace nav_start
