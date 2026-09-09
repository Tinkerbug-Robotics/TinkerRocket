#pragma once
// ─── World Magnetic Model 2025 — magnetic declination from GPS position ──────
//
// Computes magnetic declination (the angle from true north to magnetic north)
// so the EKF heading-only magnetometer update can output a TRUE-north heading.
// #1304: also exposes the FULL field vector.  The magnetometer validity gate
// compares the measured field's MAGNITUDE against what the model says it
// should be here and now — that is the only attitude-free test of whether a
// hard-iron offset has actually been removed, and it is what the old fixed
// 15–80 µT window was standing in for badly.
//
// Coefficients: NOAA/NCEI WMM2025, epoch 2025.0, Schmidt semi-normalized.
//   Source : https://www.ncei.noaa.gov/products/world-magnetic-model
//   Valid  : 2025.0 – 2030.0.
//
// ⚠  MODEL EXPIRES 2030.0.  Refresh the coefficient table from the next WMM
//    release (WMMyyyy.COF) before then; declination drifts otherwise.
//
// Off the flight-loop hot path: evaluate once when a good GPS fix is acquired,
// cache the result, and feed it to GpsInsEKF::setDeclination().

#include <compat.h>

namespace TR_GeoMag {

// Magnetic declination (radians, EAST-positive) at a geodetic position.
//   lat_rad, lon_rad : geodetic latitude / longitude (radians)
//   alt_m            : geodetic altitude above the WGS84 ellipsoid (metres)
//   decimal_year     : e.g. 2026.45; clamped to the model's valid range.
float declinationRad(double lat_rad, double lon_rad, double alt_m, double decimal_year);

/// Full geomagnetic field in the local geodetic NED frame, in MICROTESLA.
///   out_ned_uT[0] = north, [1] = east, [2] = down (down is positive in the
///   northern hemisphere, where the field dips into the ground).
/// Same arguments and same clamping as declinationRad(); the declination is
/// just atan2(east, north) of this vector, and |out| is the total intensity
/// the magnetometer should read once its hard iron is removed.
void fieldNED_uT(double lat_rad, double lon_rad, double alt_m, double decimal_year,
                 float out_ned_uT[3]);

constexpr double WMM_EPOCH     = 2025.0;
constexpr double WMM_VALID_MIN = 2025.0;
constexpr double WMM_VALID_MAX = 2030.0;

}  // namespace TR_GeoMag
