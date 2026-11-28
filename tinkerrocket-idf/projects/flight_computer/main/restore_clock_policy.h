#pragma once

#include <stdint.h>

// #1777: the flight clock a restore in flight resumes on.
//
// The flight snapshot keeps its times as elapsed from launch
// (flight_elapsed_ms, apogee_elapsed_ms, burnout_elapsed_ms;
// buildFlightSnapshot). The restore puts them back on this boot's clock,
// time_ms(), which counts from 0 at this boot's early start, whatever the
// reset was: the IDF resets the system timer as it starts it
// (esp_timer_impl_early_init, on the ESP32-P4 and the ESP32-S3).
//
// The restore used to rebase on its own moment, launch = now - flight_elapsed.
// Everything this boot did before the restore then dropped out of the
// flight's clock: the base setup, the receiver's bring-up and the asks to the
// OC. #1777 estimates about 1-2 s on the V10 board with a raw receiver's
// hint, and seconds to tens of seconds where the receiver comes up first. The
// restored apogee sat that much after the real one, and a time-after-apogee
// channel still owed at the reset fired that much late.
//
// Rebased on this boot's start instead, the clock misses only what this
// boot's clock cannot see: the snapshot's age at the reset, the reset itself,
// and the bootloader and the app's start before the timer runs. The age is
// under the 100 ms snapshot period while the flight loop runs, but seconds
// when the task watchdog resets a hung loop. The clock runs late by those and
// is never early, so no time-based channel fires before its time.
//
// The launch time lands before 0 on this boot's clock and wraps as an
// unsigned, as it already did whenever the flight was older than the boot at
// the restore. Every reader takes a difference from it. The signed ones
// (sdr_phase::sinceMs, pwr_hold::ocAliveSinceLaunch) hold for 24 days. The
// LANDED power-hold release counts an OC answer only from the restore
// (pwr_hold::answerFromMs), since every answer of this boot's setup would
// otherwise read as from after launch.
//
// Pure so the host tests can step it (tests_cpp/test_restore_clock.cpp).

namespace RestoreClock {

// This boot's start, on time_ms()'s clock.
inline constexpr uint32_t kBootStartMs = 0;

// The launch time, on this boot's clock, of a flight whose snapshot had
// flight_elapsed_ms since launch.
inline constexpr uint32_t launchMs(uint32_t flight_elapsed_ms)
{
    return kBootStartMs - flight_elapsed_ms;
}

// An event the snapshot timed from launch (apogee, burnout), on this boot's
// clock.
inline constexpr uint32_t eventMs(uint32_t launch_ms, uint32_t since_launch_ms)
{
    return launch_ms + since_launch_ms;
}

// The time-after-apogee trigger, as servicePyroChannels tests it: due once
// delay_s has passed since apogee on this boot's clock.
inline bool timeAfterApogeeDue(uint32_t now_ms, uint32_t apogee_ms, float delay_s)
{
    const float elapsed_s = (float)(now_ms - apogee_ms) / 1000.0f;
    return elapsed_s >= delay_s;
}

}  // namespace RestoreClock
