#pragma once

#include <stdint.h>

#include "RocketComputerTypes.h"

// Rescue mode: a flight computer that cannot finish booting can still take a
// firmware update.
//
// Every FC image arrives through the out computer (app -> BLE -> OC -> I2C
// OTA_BEGIN_PENDING -> I2S image pump), and the FC serves that relay only from
// loop_fc(), i.e. only once setup_fc() has finished. So an image whose setup
// stalls (a driver that never returns) or crash-loops (a panic or watchdog
// reset on every boot) cannot be replaced over the air, and with the rocket
// assembled USB means taking the airframe apart. Bench 2026-10-16: a V9 parked
// on "Starting sensors" after a battery cycle, every setting locked out and no
// way to send it the next image.
//
// IDF app rollback covers part of this and no more. It reverts an image that
// resets before it marks itself valid (#1123), so it misses an image that
// validated on one boot and stops booting on a later one, and an image that
// stalls without resetting never triggers it at all.
//
// Two mechanisms close the gap. This header holds their decisions.
//
//  1. The setup watchdog. setup_fc() still running after kSetupStallMs is a
//     stall, not a slow boot, so the FC records it and restarts. On an image
//     that has not validated yet, that restart is also what lets the
//     bootloader roll it back: a stall used to hold the rollback off forever.
//
//  2. Rescue mode. When the previous boot stalled, or kMaxUnprovenBoots boots
//     in a row never ran loop_fc() for kProvenRunMs, setup_fc() stops right
//     after the OC links are up and runs a loop that serves the OTA relay and
//     nothing else. It never initialises sensors, GNSS, servos or the camera,
//     never runs the state machine, and never arms or fires a pyro channel.
//
// Rescue mode must never cost a flight. A normal boot can carry on a flight in
// exactly one way: the OC serves it a valid in-flight snapshot (#846, #1176),
// and without one it starts cold. So before entering rescue mode the FC asks
// the OC, the same way the normal boot does, and boots normally whenever the
// OC hands back a flight or does not answer at all (a silent OC cannot relay an
// update anyway). Rescue mode is entered only when the OC is demonstrably alive
// and has no flight to hand back, which is the one case in which the normal
// boot would also have started cold.
//
// The record lives in RTC memory, which survives every reset except a power-on.
// A power-on, brownout, reset pin or USB reset is a person or the supply
// restarting the board and starts the counts from zero, so a power cycle is how
// an operator leaves rescue mode without an update. The counts belong to one
// image (its partition), so a rollback starts fresh too, and an installed
// update clears them before it restarts.
//
// Pure: main.cpp maps esp_reset_reason(), keeps the record in RTC memory, asks
// the OC and runs the loop.

namespace FcRescuePolicy {

// setup_fc() still running after this is stalled. A healthy boot measured
// 25.4 s (bench 2026-08-19); the longest legitimate one is a dead GNSS module
// (begin() blocks to its ~35 s deadline) on top of the rest, about 50 s. Over
// twice that, so a slow boot is never mistaken for a stuck one.
inline constexpr uint32_t kSetupStallMs = 120000;

// A boot that ran loop_fc() this long worked. Longer than an FC update takes
// end to end (the image at the app's 20 kB/s relay cap, plus the erase and the
// verify), so a boot that dies before an update could have finished keeps
// counting toward rescue mode.
inline constexpr uint32_t kProvenRunMs = 60000;

// Unproven boots in a row before rescue mode. Not one: a single panic on the
// pad must not take the vehicle out of its flight software.
inline constexpr uint8_t kMaxUnprovenBoots = 3;

inline constexpr uint32_t kMagic = 0x52534355;   // "RSCU"

// What restarted the board, as far as the counts care.
enum class ResetClass : uint8_t
{
    FreshStart,   // power-on, brownout, reset pin, USB/JTAG: a person or the supply
    Abnormal,     // panic, a watchdog, a software restart: the last boot ended itself
};

// RTC_NOINIT in main.cpp. A cold boot leaves garbage here, which is why every
// read goes through valid() and a FreshStart never reads it at all.
struct Record
{
    uint32_t magic;
    uint32_t image_id;        // the partition these counts belong to
    uint8_t  unproven_boots;  // consecutive boots that never ran loop_fc() for kProvenRunMs
    uint8_t  setup_stalled;   // the setup watchdog ended the previous boot
    uint16_t check;
};

inline uint16_t checkOf(const Record& r)
{
    const uint32_t x = r.magic ^ r.image_id ^
                       ((uint32_t)r.unproven_boots << 8) ^ (uint32_t)r.setup_stalled;
    return (uint16_t)(~((x >> 16) ^ (x & 0xFFFFu)) & 0xFFFFu);
}

inline Record sealed(Record r)
{
    r.magic = kMagic;
    r.check = checkOf(r);
    return r;
}

inline Record freshRecord(uint32_t image_id)
{
    Record r{};
    r.image_id = image_id;
    return sealed(r);
}

inline bool valid(const Record& r)
{
    return r.magic == kMagic && r.check == checkOf(r);
}

enum class BootMode : uint8_t
{
    Normal,
    RescueCandidate,   // rescue mode, if the OC confirms there is no flight to carry on
};

enum class Reason : uint8_t
{
    None,
    SetupStalled,      // the previous boot hit the setup watchdog
    BootsUnproven,     // kMaxUnprovenBoots boots in a row never got to kProvenRunMs
};

struct BootDecision
{
    BootMode mode;
    Reason   reason;
    Record   record;   // store this before going on
};

// Called once, first thing in setup_fc(). A Normal boot is counted here, before
// it has a chance to fail; it is uncounted when it proves itself (onProven).
inline BootDecision atBoot(const Record& stored, ResetClass reset, uint32_t image_id)
{
    const bool keep = reset == ResetClass::Abnormal && valid(stored) &&
                      stored.image_id == image_id;
    Record r = keep ? stored : freshRecord(image_id);

    if (r.setup_stalled != 0)
        return {BootMode::RescueCandidate, Reason::SetupStalled, r};
    if (r.unproven_boots >= kMaxUnprovenBoots)
        return {BootMode::RescueCandidate, Reason::BootsUnproven, r};

    r.unproven_boots = (uint8_t)(r.unproven_boots + 1);
    return {BootMode::Normal, Reason::None, sealed(r)};
}

// The OC's answers, asked before rescue mode is allowed.
struct OcCheck
{
    bool answered;          // the OC answered a status query this boot
    bool flight_to_resume;  // it served a valid, non-sim, in-flight snapshot
};

inline bool mayEnterRescue(const BootDecision& d, const OcCheck& oc)
{
    return d.mode == BootMode::RescueCandidate && oc.answered && !oc.flight_to_resume;
}

// A candidate the OC turned down boots normally, and is counted like any
// normal boot. The stall flag is spent: it described the previous boot, and
// the watchdog sets it again if this one stalls too.
inline Record onRescueRefused(Record r)
{
    r.setup_stalled = 0;
    if (r.unproven_boots < 0xFFu) r.unproven_boots = (uint8_t)(r.unproven_boots + 1);
    return sealed(r);
}

// The setup watchdog fired: the next boot is a rescue candidate.
inline Record onSetupStall(Record r)
{
    r.setup_stalled = 1;
    return sealed(r);
}

// loop_fc() has run kProvenRunMs: this image boots.
inline Record onProven(Record r)
{
    r.unproven_boots = 0;
    r.setup_stalled = 0;
    return sealed(r);
}

// An update is about to restart into a new image, which starts fresh. Its
// partition differs, so atBoot() would start it fresh anyway; clearing here
// also covers this image coming back after a rollback.
inline Record onUpdateInstalled(Record r)
{
    return onProven(r);
}

inline bool provenAfter(uint32_t loop_run_ms)
{
    return loop_run_ms >= kProvenRunMs;
}

// The OC commands rescue mode serves. Everything else is dropped (and said so
// in the log): configs, cals and tests all need the flight software that
// rescue mode does not start, and the one-shot actuating commands (#1105) are
// exactly what it must never act on.
inline bool servesInRescue(uint8_t cmd)
{
    return cmd == OTA_BEGIN_PENDING || cmd == OTA_FINISH_CMD || cmd == OTA_ABORT_CMD;
}

}  // namespace FcRescuePolicy
