#pragma once

#include <stdint.h>

// Is the camera running? Read from the camera rail's current (#850), not from
// the camera, and reported to the FC in the status byte
// (OUT_STATUS_CAM_MONITORED_BIT / OUT_STATUS_CAM_DRAWING_BIT).
//
// CENJARS 2026-11-01: neither GTV RunCam ever answered the FC's readiness probe.
// Both were running (each recorded on its own from power-up), but the FC could
// only see silence, so it walked its power-cycle ladder: power-on, a silent boot
// window, 3 s off, twice. Each cut ended a recording, and White's last power-on
// came about 1 s before liftoff, so its video missed the boost. The ladder exists
// for a camera that comes up DARK (the V8 battery cold start), and the rail
// current tells dark from running without the camera's cooperation.
//
// Camera rail current, A, from the 10 Hz monitor reads in the logs:
//   gate off                0.000
//   RunCam booting          0.085   (one sample)
//   RunCam recording        0.10-0.32 (GTV Blue/White in flight, p1 0.11-0.14)
//   GoPro booted, idle      0.17-0.35
//   GoPro recording         0.59-1.09
// A RunCam that is powered but NOT recording was not seen: these units record
// at power-up. Running vs dark is the whole question here, and 50 mA sits well
// clear of both sides. The monitor's gain spreads +-13 % over temperature
// (#850), which these margins absorb uncalibrated.
//
// Hysteresis plus a half-second debounce at the 10 Hz read, so a boot-current
// dip or one noisy read cannot flip the answer. A read that is not a number (no
// monitor, or the ADC failed) drops both facts at once: "unknown" never reads
// as "dark", which would bring back the power-cycle this exists to avoid.
//
// Pure so the thresholds and the debounce are host-testable, in the house style
// of supply_status_policy.h.

namespace CamDrawPolicy {

constexpr float   kOnAmps  = 0.050f;  // at or above: counts toward running
constexpr float   kOffAmps = 0.030f;  // below: counts toward dark
constexpr uint8_t kReads   = 5;       // consecutive reads to change state (0.5 s at 10 Hz)

class Debounce
{
public:
    // One camera-rail reading, in amps; NaN when the monitor did not read.
    void update(float amps)
    {
        if (!(amps == amps))
        {
            monitored_ = false;
            drawing_ = false;
            run_ = 0;
            return;
        }
        monitored_ = true;
        const bool toward_on  = !drawing_ && amps >= kOnAmps;
        const bool toward_off =  drawing_ && amps <  kOffAmps;
        if (!(toward_on || toward_off))
        {
            run_ = 0;
            return;
        }
        if (++run_ >= kReads)
        {
            drawing_ = !drawing_;
            run_ = 0;
        }
    }

    // The last reading was a number.
    bool monitored() const { return monitored_; }
    // Running current for the last kReads reads (held through the hysteresis band).
    bool drawing() const { return monitored_ && drawing_; }

private:
    bool    monitored_ = false;
    bool    drawing_   = false;
    uint8_t run_       = 0;
};

}  // namespace CamDrawPolicy
