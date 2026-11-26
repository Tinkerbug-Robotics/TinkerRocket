#pragma once

#include <stdint.h>

#include <RocketComputerTypes.h>   // RocketState

#include "nav_start_policy.h"   // #1762: the start rule, now every receiver's

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
// #1762: the rule above now covers every receiver (nav_start_policy.h); this
// header keeps what is the board's alone (its boot check) and its own names
// for the shared rule, the board being a receiver that never fails bring-up.
// Pure, host-tested (tests_cpp/test_sdr_nav_start_policy.cpp).
namespace sdr_nav_start
{

static constexpr uint32_t kNoFixMs       = nav_start::kNoFixMs;
static constexpr uint32_t kHeardMs       = 3000u;   // the boot waits this long for the board's first frame
static constexpr uint32_t kUsableFixMs   = nav_start::kUsableFixMs;
static constexpr uint32_t kReinitGraceMs = nav_start::kReinitGraceMs;

inline bool boardWait(uint32_t last_usable_ms, uint32_t reinit_ms, uint32_t now_ms)
{
    return nav_start::receiverWait(last_usable_ms, reinit_ms, now_ms);
}

inline bool degradedStartDue(bool normal_ready, bool gyro_ready, bool settled, uint32_t since_boot_ms,
                             RocketState state)
{
    return nav_start::degradedStartDue(normal_ready, gyro_ready, settled, since_boot_ms, state,
                                       /*receiver_absent=*/false);
}

using nav_start::restartOnFix;

// The boot's verdict on the board: silent once kHeardMs passed with no frame.
inline bool boardSilent(bool heard, uint32_t since_sensors_ms)
{
    return !heard && since_sensors_ms >= kHeardMs;
}

}  // namespace sdr_nav_start
