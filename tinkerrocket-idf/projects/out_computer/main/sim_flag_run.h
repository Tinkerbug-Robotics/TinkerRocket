#pragma once

#include <stdint.h>

// #1774: when the OC's two flight-safety decisions believe the FC's
// NSF_SIM_ACTIVE.
//
// Two OC decisions take the flag as proof that the FC is flying a simulation:
//   - arm consent (arm_consent_policy.h): a sim's INFLIGHT holds OC_ARM_EN
//     low, latched for the rest of the phase;
//   - the post-landing log lockout (landed_lockout_policy.h): a sim's landing,
//     or a sim's start, clears it.
// Each acted on a single frame. The frame's only integrity check is the
// link's CRC-16, which has Hamming distance 2 on a NonSensorData frame (#1729).
// Flipping flags bit 7 (the flag) and payload byte 6 bit 6 passes it: the two
// bits are 255 apart, the order of x modulo x^16+x^15+1. So do some 3-bit
// errors. One such frame between launch and the last deployment would latch
// "sim" and hold consent low until LANDED. The FC's pulses would then meet an
// open U9, and each channel would go Done unfired.
//
// The FC never sets the flag for one frame. Since #1766 it reports it for the
// whole of a sim's flight state:
//   - from the Start edge on the pad, through the sim's 5 s pad phase, which
//     starts there (the FC's 3 s GNSS dwell in READY runs inside it);
//   - through its INFLIGHT and LANDED;
//   - until the reset of the operator's Stop.
// So these two decisions take the flag as set only once kFrames frames in a
// row have carried it, and a frame without it starts the count again. A link
// error the CRC misses changes one frame, not ten in a row. Frames that fail
// the CRC, and the re-delivered frames processFrame's time_us filter drops,
// are never fed here, so they neither break nor fill a run.
//
// The cost is kFrames - 1 frames (18 ms at the FC's 500 Hz) read as a real
// flight, with consent up while the FC dry-fires, in two cases:
//   - an OC that joins a sim's flight in progress (an OC reboot during a
//     bench sim);
//   - a sim frame whose flag a missed link error cleared in the last
//     kFrames - 1 frames before the sim's launch.
//
// The OC's other readers of the flag still act on one frame: the stray-Stop
// gate, the blind-window check and the telemetry. None of them changes
// consent or a deployment.
//
// Every frame is fed in arrival order, once (processFrame). Pure, so
// tests_cpp/test_sim_flag_run.cpp can run the issue's frame through it.

namespace SimFlagRun {

// Frames in a row that must carry the flag: 20 ms at the FC's 500 Hz
// NonSensorData rate (flight_computer config.h NON_SENSOR_UPDATE_RATE).
inline constexpr uint8_t kFrames = 10;

struct Run
{
    uint8_t n = 0;   // frames in a row with the flag, held at kFrames
};

// Feed one frame's NSF_SIM_ACTIVE bit. Returns whether the flag is now set on
// the last kFrames frames.
inline bool step(Run& r, bool sim_active)
{
    if (!sim_active)
    {
        r.n = 0;
        return false;
    }
    if (r.n < kFrames) r.n++;
    return r.n >= kFrames;
}

}  // namespace SimFlagRun
