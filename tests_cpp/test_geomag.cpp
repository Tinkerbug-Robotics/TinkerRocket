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

// ===========================================================================
// #1304 — the full field vector.  The magnetometer validity gate compares the
// measured magnitude against the model's total intensity, so the model's
// MAGNITUDE and DIP now have to be right, not only its declination (which the
// cases above already pin against published values).
//
// The anchor is BARC, and it is not a looked-up number: the four 2026-08-29
// flights measured it. Sphere fits of their own magnetometer samples give a
// field radius of 47.9–51.8 µT. The pad check — the angle between the
// hard-iron-corrected field and the accelerometer's vertical, which needs no
// model at all — gives inclinations of 66.5, 62.3, 67.3 and 61.5°, a mean of
// 64.4° with 2.9° of flight-to-flight scatter. The model has to land inside
// that, and it does: 64.8°, 0.4° from the mean.
// ===========================================================================
namespace field1304 {
static constexpr double BARC_LAT = 39.4689, BARC_LON = -75.2918;
static constexpr double YEAR = 2026.7;
static void fieldAt(double lat_deg, double lon_deg, double alt_m, float out[3]) {
    TR_GeoMag::fieldNED_uT(lat_deg * M_PI / 180.0, lon_deg * M_PI / 180.0,
                           alt_m, YEAR, out);
}
static float totalOf(const float b[3]) {
    return std::sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
}
static float inclDegOf(const float b[3]) {
    return (float)(std::atan2(b[2], std::hypot(b[0], b[1])) * 180.0 / M_PI);
}
}  // namespace field1304
using namespace field1304;

TEST(GeoMagField1304, BarcMatchesWhatTheFlightsMeasured) {
    float b[3];
    fieldAt(BARC_LAT, BARC_LON, 0.0, b);
    EXPECT_NEAR(totalOf(b),   49.9f, 2.0f);   // sphere fits: 47.9–51.8 µT
    EXPECT_NEAR(inclDegOf(b), 64.4f, 1.5f);   // pad dip check, mean of four flights
}

TEST(GeoMagField1304, DeclinationIsTheHorizontalAtan2OfTheSameVector) {
    // One evaluation feeds both entry points; they must not drift apart.
    const double pts[][2] = {{BARC_LAT, BARC_LON}, {40.015, -105.2705},
                             {-33.8688, 151.2093}, {0.0, 10.0}, {78.0, 15.0}};
    for (const auto& p : pts) {
        float b[3];
        fieldAt(p[0], p[1], 0.0, b);
        const float direct = TR_GeoMag::declinationRad(
            p[0] * M_PI / 180.0, p[1] * M_PI / 180.0, 0.0, YEAR);
        EXPECT_NEAR(std::atan2(b[1], b[0]), direct, 1e-6f) << p[0] << "," << p[1];
    }
}

TEST(GeoMagField1304, TotalIntensityStaysInsideEarthsActualRange) {
    // Earth's surface field runs roughly 22–67 µT. The gate's tolerance is a
    // FRACTION of this number, so a model that returned nT, or zero, or a
    // wildly wrong magnitude would silently disable or wreck the mag update.
    for (double lat = -85.0; lat <= 85.0; lat += 5.0) {
        for (double lon = -180.0; lon < 180.0; lon += 15.0) {
            float b[3];
            fieldAt(lat, lon, 0.0, b);
            const float t = totalOf(b);
            EXPECT_GT(t, 20.0f) << lat << "," << lon;
            EXPECT_LT(t, 70.0f) << lat << "," << lon;
        }
    }
}

TEST(GeoMagField1304, DipFollowsTheHemisphereAndFlipsAtTheMagneticEquator) {
    float b[3];
    fieldAt(60.0, -75.0, 0.0, b);   EXPECT_GT(inclDegOf(b), 60.0f);   // far north: steep down
    fieldAt(-60.0, -75.0, 0.0, b);  EXPECT_LT(inclDegOf(b), -50.0f);  // far south: steep up
    // Walking north along 10°E the dip must cross zero exactly once, and the
    // crossing is the magnetic equator — north of the geographic one there.
    int crossings = 0;
    float prev = 0.0f;
    for (double lat = -30.0; lat <= 30.0; lat += 1.0) {
        fieldAt(lat, 10.0, 0.0, b);
        const float d = inclDegOf(b);
        if (lat > -30.0 && ((prev < 0.0f) != (d < 0.0f))) crossings++;
        prev = d;
    }
    EXPECT_EQ(crossings, 1);
}

TEST(GeoMagField1304, AltitudeWeakensTheFieldMonotonically) {
    float prev = 1e9f;
    for (double alt = 0.0; alt <= 30000.0; alt += 2000.0) {
        float b[3];
        fieldAt(BARC_LAT, BARC_LON, alt, b);
        const float t = totalOf(b);
        EXPECT_LT(t, prev) << "alt " << alt;
        prev = t;
    }
    // Over a rocket's altitude band the change is small enough that the gate
    // does not have to re-evaluate the model per sample.
    float sea[3], high[3];
    fieldAt(BARC_LAT, BARC_LON, 0.0, sea);
    fieldAt(BARC_LAT, BARC_LON, 10000.0, high);
    EXPECT_NEAR(totalOf(high), totalOf(sea), 0.05f * totalOf(sea));
}
