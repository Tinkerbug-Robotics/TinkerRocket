// GnssClockRecord.h — GNSS_CLOCK_MSG's record from GnssTimeSync's line
//
// The driver logs the line each time it moves (GnssTimeSync::lineSeq), so a
// replay places each raw epoch and fix on the IMU's clock exactly as the
// flight did (RocketComputerTypes.h, GnssClockData). The flight loop, which
// has the record and not the driver's GnssTimeSync, reads GPS time off it
// the same way (gnssClockGpsAt). Pure and host-testable
// (tests_cpp/test_gnss_time_sync.cpp).
#ifndef GNSS_CLOCK_RECORD_H
#define GNSS_CLOCK_RECORD_H

#include <cstdint>
#include <cstring>

#include <RocketComputerTypes.h>

#include "GnssTimeSync.h"

namespace lc86
{

// time_us: the MCU micros the line took effect (the backdated end of the
// $PQTMPVT that moved it).
inline void gnssClockRecord(const GnssTimeSync::Snapshot& s, uint32_t time_us, GnssClockData& out)
{
    std::memset(&out, 0, sizeof(out));
    out.time_us = time_us;
    out.seq = s.seq;
    out.gps0_s = static_cast<uint32_t>(s.gps0_s);
    out.mcu0_us = s.mcu0_us;
    out.a_s = s.a_s;
    out.t_ref_s = s.t_ref_s;
    out.drift = s.drift;
    out.latency_s = s.latency_s;
    out.env_rms_ms = static_cast<float>(s.env_rms_ms);
    out.late90_ms = static_cast<float>(s.late90_ms);
    out.samples = s.samples;
    out.restarts = static_cast<uint16_t>(s.restarts > 65535u ? 65535u : s.restarts);
    out.bins = static_cast<uint8_t>(s.bins < 0 ? 0 : (s.bins > 255 ? 255 : s.bins));
    out.flags = static_cast<uint8_t>((s.valid ? GCL_VALID : 0) | (s.started ? GCL_STARTED : 0));
}

// The GPS time (continuous seconds, week * 604800 + time of week) of an MCU
// instant on a logged line: GnssTimeSync::gpsAt from the record alone, the
// line's formula (RocketComputerTypes.h, GnssClockData) turned around:
//   t = ((mcu_us - mcu0_us) * 1e-6 - a_s + drift * t_ref_s + latency_s) / (1 + drift);  gps_s = gps0_s + t.
// False unless GCL_VALID.
inline bool gnssClockGpsAt(const GnssClockData& c, int64_t mcu_us, double& gps_s)
{
    if ((c.flags & GCL_VALID) == 0) return false;
    const double mcu_rel = static_cast<double>(mcu_us - c.mcu0_us) * 1.0e-6;
    const double t = (mcu_rel - c.a_s + c.drift * c.t_ref_s + c.latency_s) / (1.0 + c.drift);
    gps_s = static_cast<double>(c.gps0_s) + t;
    return true;
}

}  // namespace lc86

#endif  // GNSS_CLOCK_RECORD_H
