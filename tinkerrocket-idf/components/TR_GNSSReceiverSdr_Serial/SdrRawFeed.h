// SdrRawFeed.h — the gnss-sdr-dev board's stream into what the raw-GNSS
// navigation filter and the flight loop take: pure and header-only, so the
// flight computer's driver (TR_GNSSReceiverSdrSerial::enableRawOutputs, the
// V10's SDR raw-nav image) and tools/raw_replay's --receiver sdr run the same
// lines on the same frames.
//
// Per frame, in this order (the driver's onFrame, raw_replay's 'R' lines):
//   1. a 4001 sub-type 1 gives the clock line its sample (lineSampleOf): the
//      GPS time the board latched the epoch at -- the MSMs' tag less the
//      record's sub-ms shift (sdr_aid::latchGps) -- against when the record's
//      last byte arrived; none from an epoch the board marks late (its
//      arrival carries the board's wait, not its delay);
//   2. the frame to SdrEpochAssembler (SdrRawEpoch.h), which hands over the
//      epochs on the fused grid, their Dopplers the window's means where the
//      assembler averages them;
//   3. each epoch handed over placed on the MCU clock by the line as it
//      stands after its own record's sample (timedEpochOf): its GPS instant
//      is the tag, where every observable was carried to.
// The board's fix (4001 sub-type 2) becomes the flight computer's GNSSData
// (gnssDataFromTrFix): the raw filter's start position and the GPS apogee
// voter's, the pad average's and the telemetry's fix. Its height is above
// the WGS84 ellipsoid -- the board gives no geoid -- and the driver says so
// (geoidSeparationM: 0), so every height the flight computer turns between
// the two is the ellipsoid's.
#ifndef SDR_RAW_FEED_H
#define SDR_RAW_FEED_H

#include <cmath>
#include <cstdint>
#include <cstring>

#include <RocketComputerTypes.h>   // GNSSData
#include "GnssTimeSync.h"          // TR_GNSSReceiverLC86_Serial: the line
#include "RawMeas.h"               // TR_GNSSRaw: TimedRawEpoch
#include "SdrAid.h"                // decodeEpochAid, latchGps
#include "SdrRawEpoch.h"

namespace sdr_rtcm
{

// 1. The clock line's sample a 4001 sub-type 1 gives (rec decoded from the
// same frame): its latch's GPS time (week * 604800 + time of week, s). False
// for a frame that is no such record, or one the board marks late.
inline bool lineSampleOf(const uint8_t* frame, size_t len, const TrEpoch& rec, double& gps_s)
{
    sdr_aid::EpochAid aid;
    if (!sdr_aid::decodeEpochAid(frame, len, aid))
        return false;
    return sdr_aid::latchGps(rec, aid, gps_s);
}

// 3. An epoch the assembler handed over, as the filter takes it: its GPS
// instant (the tag) on the MCU clock where the line is valid, else mcu_us 0
// (untimed: the filter waits for a timed one); its time of week the board's
// unstepped clock's (SdrRawEpoch.h), tag_dtr_ms how far that clock is ahead
// of GPS time (tow_s - tag_dtr_ms / 1000 is the tag, which the rig replay's
// clock takes); rr_paired its Dopplers' means' mark (the filter takes them at
// the window's middle). True when it was timed.
inline bool timedEpochOf(const SdrEpoch& e, const lc86::GnssTimeSync& line, gnss_raw::TimedRawEpoch& out)
{
    out.epoch = e.epoch;
    out.arrival_us = e.arrival_us;
    out.gps_week = e.week;
    out.rr_paired = e.rr_mean;
    int64_t mcu = 0;
    const bool timed = line.valid() && line.mcuAt(e.gps_s, mcu);
    out.mcu_us = timed ? mcu : 0;
    out.tag_dtr_ms = timed ? static_cast<float>(e.step_m / kC * 1.0e3) : 0.0f;
    return timed;
}

// WGS-84 ECEF (m) to latitude, longitude (rad) and height above the ellipsoid
// (m), and the turn from ECEF to the local NED at that latitude and longitude.
inline void ecefToLla(const double p[3], double lla[3])
{
    static constexpr double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
    const double r = std::hypot(p[0], p[1]);
    double lat = std::atan2(p[2], r * (1.0 - e2)), h = 0.0;
    for (int i = 0; i < 8; ++i)
    {
        const double sl = std::sin(lat);
        const double n = a / std::sqrt(1.0 - e2 * sl * sl);
        h = r / std::cos(lat) - n;
        lat = std::atan2(p[2], r * (1.0 - e2 * n / (n + h)));
    }
    lla[0] = lat;
    lla[1] = std::atan2(p[1], p[0]);
    lla[2] = h;
}
inline void ecefToNed(double lat, double lon, const double v[3], double ned[3])
{
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    ned[0] = -sl * co * v[0] - sl * so * v[1] + cl * v[2];
    ned[1] = -so * v[0] + co * v[1];
    ned[2] = -cl * co * v[0] - cl * so * v[1] - sl * v[2];
}

// GPS week and time of week (ms) as UTC, leap_s behind (18 since 2017).
inline void gpsToUtcFields(uint16_t week, uint32_t tow_ms, int leap_s, GNSSData& out)
{
    const int64_t ms = static_cast<int64_t>(week) * 604800000LL + tow_ms - static_cast<int64_t>(leap_s) * 1000LL;
    int64_t days = ms / 86400000LL;
    const int64_t in_day = ms % 86400000LL;
    // days since 1970-01-01: GPS time starts 1980-01-06, 3657 days later
    days += 3657;
    // civil from days (H. Hinnant's algorithm)
    days += 719468;
    const int64_t era = days / 146097;
    const int64_t doe = days - era * 146097;
    const int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const int64_t mp = (5 * doy + 2) / 153;
    const int64_t d = doy - (153 * mp + 2) / 5 + 1;
    const int64_t m = mp < 10 ? mp + 3 : mp - 9;
    const int64_t y = yoe + era * 400 + (m <= 2 ? 1 : 0);
    out.year = static_cast<uint16_t>(y);
    out.month = static_cast<uint8_t>(m);
    out.day = static_cast<uint8_t>(d);
    out.hour = static_cast<uint8_t>(in_day / 3600000LL);
    out.minute = static_cast<uint8_t>((in_day / 60000LL) % 60);
    out.second = static_cast<uint8_t>((in_day / 1000LL) % 60);
    out.milli_second = static_cast<uint16_t>(in_day % 1000LL);
}

static constexpr int kSdrLeapSeconds = 18;   // GPS - UTC since 2017-01-01 (the board sends none)

inline int32_t sdrRoundI32(double v)
{
    if (!(v > -2147483648.0)) return INT32_MIN;
    if (!(v < 2147483647.0)) return INT32_MAX;
    return static_cast<int32_t>(std::llround(v));
}

// The board's fix as the flight computer's GNSSData, stamped time_us (its
// record's arrival). True only for a fix the flight computer can use as one:
// valid, with its velocity (GNSSData has no flag for a fix without one, and
// a zero there would read as a stop at apogee to the GPS voter). The time
// fields are filled either way; out is zeroed first. Height above the
// ellipsoid (see the top); no accuracy given (h_acc_m, v_acc_m 255).
inline bool gnssDataFromTrFix(const TrFix& f, uint32_t time_us, GNSSData& out)
{
    std::memset(&out, 0, sizeof(out));
    out.time_us = time_us;
    gpsToUtcFields(f.week, f.tow_ms, kSdrLeapSeconds, out);
    out.h_acc_m = 255;
    out.v_acc_m = 255;
    out.num_sats = f.nsat;
    const double pdop = std::isfinite(f.pdop) ? f.pdop : 0.0;
    const long pd = std::lround(pdop * 10.0);
    out.pdop_x10 = static_cast<uint8_t>(pd < 0 ? 0 : (pd > 255 ? 255 : pd));
    const double r = std::sqrt(f.pos[0] * f.pos[0] + f.pos[1] * f.pos[1] + f.pos[2] * f.pos[2]);
    if (!f.valid || !f.vel_valid || !(r > 6.0e6) || !(r < 6.6e6))
        return false;   // fix_mode 0: the time alone
    double lla[3], ned[3];
    ecefToLla(f.pos, lla);
    ecefToNed(lla[0], lla[1], f.vel, ned);
    static constexpr double kRad2Deg = 180.0 / M_PI;
    out.fix_mode = 3;
    out.lat_e7 = sdrRoundI32(lla[0] * kRad2Deg * 1.0e7);
    out.lon_e7 = sdrRoundI32(lla[1] * kRad2Deg * 1.0e7);
    out.alt_mm = sdrRoundI32(lla[2] * 1000.0);
    out.vel_n_mmps = sdrRoundI32(ned[0] * 1000.0);
    out.vel_e_mmps = sdrRoundI32(ned[1] * 1000.0);
    out.vel_u_mmps = sdrRoundI32(-ned[2] * 1000.0);
    return true;
}

}  // namespace sdr_rtcm

#endif  // SDR_RAW_FEED_H
