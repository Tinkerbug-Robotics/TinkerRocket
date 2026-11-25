#pragma once

#include <stdint.h>

#include <RocketComputerTypes.h>   // RocketState

// The V10's gnss-sdr-dev image (TR_RAW_NAV_SDR): when its one navigation
// filter starts without the board, and when a late board takes it over.
//
// The board's driver comes up whether or not the board is there (begin() opens
// the UART and listens), so the boot cannot tell a silent J1 from a board
// still on its cold start, and the image is never GNSS-absent the way the
// mosaic image is when its receiver does not answer. Waiting for a usable fix
// alone, a board that never gives one -- J1 unplugged, a hung board, a cold
// start longer than the pad -- left the filter unstarted for the flight
// ("Launching without EKF initialization"): worse than the auto image, which
// starts GNSS-absent beside a board it only logs. So:
//   - the degraded start (the raw filter on the IMU and the barometer from a
//     pad-relative 0 N 0 E, as the GNSS-absent images start) once no usable
//     fix has come kNoFixMs after boot on the pad (READY), and at once when the
//     rocket is armed (PRELAUNCH) -- never in flight, where the accelerometer
//     is no measure of the tilt;
//   - a usable fix that comes later, while the rocket is still READY (on the
//     pad, unarmed), starts the filter over on it (the normal start: a new
//     start generation restarts the raw filter from the board's position);
//     once armed or flying, the degraded start stands for the flight;
//   - while it stands, the board's epochs are held out of the filter (its
//     position is no place to build them at) and its fixes set no pad datum;
//   - the boot says FCB_DEG_GNSS when no frame of the board's 4001 was heard
//     kHeardMs after the sensors started (its status comes once a second from
//     its receiver's start);
//   - "no usable fix" means none for a while, not none on this loop pass
//     (#1756): the board gives a fix 1-10 times a second and the loop runs at
//     1 kHz, so a usable fix arrives on one pass in a hundred or more.  Any
//     re-init of a filter already flying on the board's fixes (a mag-cal exit,
//     an orientation re-snap or setting change, all allowed armed on the pad)
//     used to start degraded on the next pass between two fixes, and armed
//     that stood for the flight: the board's epochs held out, the datum
//     frozen, guidance off at launch.  The degraded start now waits while a
//     usable fix came in the last kUsableFixMs (the next one starts the
//     filter normally), and for kReinitGraceMs after such a re-init (a mag
//     cal tumbles the rocket, whose fixes then fail the init gate's low
//     velocity until it is set down; the session's exit restarts the grace)
//     -- boardWait().  A board that never gave a fix, or went silent, still
//     gets the degraded start, at most kReinitGraceMs later; and one whose
//     usable fixes do not come back within the grace after a re-init armed
//     still stands degraded for the flight, as before.
// Pure, host-tested (tests_cpp/test_sdr_nav_start_policy.cpp).
namespace sdr_nav_start
{

static constexpr uint32_t kNoFixMs = 120000u;   // no usable fix 2 min after boot: start without the board
static constexpr uint32_t kHeardMs = 3000u;     // the boot waits this long for the board's first frame
static constexpr uint32_t kUsableFixMs   = 3000u;   // a usable fix this recent: the next one starts the filter
static constexpr uint32_t kReinitGraceMs = 5000u;   // after a re-init of a filter started on the board's fixes

// The degraded start waits for the board (#1756): a usable fix (one the
// normal start would take: the init gate with the pad datum) came within
// kUsableFixMs, or a filter that had started on the board's fixes was
// re-initialized within kReinitGraceMs.  last_usable_ms / reinit_ms: millis()
// of each, 0 for never.  Ages in unsigned arithmetic, across the wrap.
inline bool boardWait(uint32_t last_usable_ms, uint32_t reinit_ms, uint32_t now_ms)
{
    if (last_usable_ms != 0 && (uint32_t)(now_ms - last_usable_ms) < kUsableFixMs)
        return true;
    return reinit_ms != 0 && (uint32_t)(now_ms - reinit_ms) < kReinitGraceMs;
}

// The degraded start is due: no usable fix to start on now and none to wait
// for (normal_ready: a normal start due this pass, or boardWait()),
// the gyro's bias averaged (gyro_ready), the GNSS-absent settle over
// (settled: main.cpp's gnss_absent_dwell_ms, 5 s after boot and after a sim's
// reset, so a sim's first fix starts it normally), and either kNoFixMs since
// boot on the pad or the rocket armed.
inline bool degradedStartDue(bool normal_ready, bool gyro_ready, bool settled, uint32_t since_boot_ms,
                             RocketState state)
{
    if (normal_ready || !gyro_ready || !settled)
        return false;
    if (state == PRELAUNCH)
        return true;
    return state == READY && since_boot_ms >= kNoFixMs;
}

// A degraded start gives way to the board's first usable fix: only on the pad,
// unarmed.
inline bool restartOnFix(bool degraded_started, bool normal_ready, RocketState state)
{
    return degraded_started && normal_ready && state == READY;
}

// The boot's verdict on the board: silent once kHeardMs passed with no frame.
inline bool boardSilent(bool heard, uint32_t since_sensors_ms)
{
    return !heard && since_sensors_ms >= kHeardMs;
}

}  // namespace sdr_nav_start
