#include <gtest/gtest.h>
#include "TR_GeoMag.h"
#include <cmath>

// Validation against the official NOAA WMM2025_TestValues.txt (field 5 =
// declination, deg).  A representative spread of epochs / latitudes /
// longitudes, including a near-pole point and a large-declination point.
namespace {
struct TV { double year, alt_km, lat_deg, lon_deg, dec_deg; };
constexpr TV kTests[] = {
    {2025.0, 28,  89, -121,  -99.77},   // near north pole
    {2025.0, 65,  43,   93,    0.50},
    {2025.0, 51, -33,  109,   -5.49},
    {2025.0, 18,   0,   21,    1.29},
    {2025.5, 63,  26,   81,    0.51},
    {2026.0, 69,  23,   63,    1.17},
    {2026.5, 12, -79,  115, -137.58},   // large declination, high south lat
    {2027.5, 16,  66, -178,    0.37},
    {2028.0, 95,  14,   65,   -0.51},
    {2029.5, 33,  17,    5,    0.89},
};
constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;
}  // namespace

TEST(GeoMagTest, DeclinationMatchesNOAAReference) {
    for (const auto& tv : kTests) {
        float dec_rad = TR_GeoMag::declinationRad(
            tv.lat_deg * D2R, tv.lon_deg * D2R, tv.alt_km * 1000.0, tv.year);
        double dec_deg = dec_rad * R2D;
        // Declination wraps at ±180; compare on the circle.
        double err = std::remainder(dec_deg - tv.dec_deg, 360.0);
        EXPECT_NEAR(err, 0.0, 0.1)
            << "year=" << tv.year << " lat=" << tv.lat_deg
            << " lon=" << tv.lon_deg << " got=" << dec_deg
            << " expected=" << tv.dec_deg;
    }
}
