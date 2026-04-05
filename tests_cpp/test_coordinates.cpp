#include <gtest/gtest.h>
#include "TR_Coordinates.h"
#include <cmath>

class CoordinatesTest : public ::testing::Test {
protected:
    TR_Coordinates coord;
};

TEST_F(CoordinatesTest, GeodeticToECEF_Equator) {
    // Lat=0, Lon=0, Alt=0 -> ECEF should be (a, 0, 0)
    double x, y, z;
    coord.geodeticToECEF(0.0, 0.0, 0.0, x, y, z); // inputs in radians
    EXPECT_NEAR(x, 6378137.0, 1.0);
    EXPECT_NEAR(y, 0.0, 0.1);
    EXPECT_NEAR(z, 0.0, 0.1);
}

TEST_F(CoordinatesTest, ECEFToGeodetic_Roundtrip) {
    // Start with known LLA (in radians for geodeticToECEF)
    double lat_rad = 33.7 * M_PI / 180.0;
    double lon_rad = -118.4 * M_PI / 180.0;
    double alt = 150.0;

    double x, y, z;
    coord.geodeticToECEF(lat_rad, lon_rad, alt, x, y, z);

    double lat_out, lon_out, alt_out;
    coord.ecefToGeodetic(x, y, z, lat_out, lon_out, alt_out);

    // ecefToGeodetic returns degrees
    EXPECT_NEAR(lat_out, 33.7, 1e-5);
    EXPECT_NEAR(lon_out, -118.4, 1e-5);
    EXPECT_NEAR(alt_out, 150.0, 0.01); // < 1cm
}

TEST_F(CoordinatesTest, ENUOrigin_SetAndConvert) {
    // Set origin to a known point
    coord.setENUOrigin(33.7, -118.4, 100.0); // degrees

    // Origin point should map to ENU (0,0,0)
    double e, n, u;
    coord.geodeticToENU(33.7, -118.4, 100.0, e, n, u);

    EXPECT_NEAR(e, 0.0, 0.1);
    EXPECT_NEAR(n, 0.0, 0.1);
    EXPECT_NEAR(u, 0.0, 0.1);
}

TEST_F(CoordinatesTest, GeodeticToENU_100mNorth) {
    coord.setENUOrigin(33.7, -118.4, 100.0);

    // ~0.0009 degrees north of origin ≈ 100m
    // 1 degree latitude ≈ 111,320 m
    double delta_lat_deg = 100.0 / 111320.0;
    double e, n, u;
    coord.geodeticToENU(33.7 + delta_lat_deg, -118.4, 100.0, e, n, u);

    EXPECT_NEAR(e, 0.0, 1.0);    // no east offset
    EXPECT_NEAR(n, 100.0, 2.0);  // ~100m north
    EXPECT_NEAR(u, 0.0, 0.5);    // same altitude
}

TEST_F(CoordinatesTest, Quat2Euler_Identity) {
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float euler[3];
    coord.Quat2Euler(q, euler);

    // Identity quaternion -> all angles zero
    EXPECT_NEAR(euler[0], 0.0f, 1e-5f); // roll
    EXPECT_NEAR(euler[1], 0.0f, 1e-5f); // pitch
    EXPECT_NEAR(euler[2], 0.0f, 1e-5f); // yaw
}

TEST_F(CoordinatesTest, Quat2Euler_90DegRoll) {
    // 90-degree roll: q = [cos(45), sin(45), 0, 0]
    float half = (float)(M_PI / 4.0);
    float q[4] = {std::cos(half), std::sin(half), 0.0f, 0.0f};
    float euler[3];
    coord.Quat2Euler(q, euler);

    EXPECT_NEAR(euler[0], (float)(M_PI / 2.0), 0.01f); // roll ≈ 90 deg
    EXPECT_NEAR(euler[1], 0.0f, 0.01f);                 // pitch ≈ 0
}

TEST_F(CoordinatesTest, SetENUOriginECEF_Roundtrip) {
    // Set origin via LLA, then verify ECEF-based set gives same result
    coord.setENUOrigin(34.0, -117.5, 200.0);

    // Get ECEF of origin by converting
    double x0, y0, z0;
    double lat_rad = 34.0 * M_PI / 180.0;
    double lon_rad = -117.5 * M_PI / 180.0;
    coord.geodeticToECEF(lat_rad, lon_rad, 200.0, x0, y0, z0);

    // Set up a second coord object using ECEF origin
    TR_Coordinates coord2;
    coord2.setENUOriginECEF(x0, y0, z0);

    // Both should produce same ENU for a test point
    double e1, n1, u1, e2, n2, u2;
    coord.geodeticToENU(34.001, -117.499, 210.0, e1, n1, u1);
    coord2.geodeticToENU(34.001, -117.499, 210.0, e2, n2, u2);

    EXPECT_NEAR(e1, e2, 0.5);
    EXPECT_NEAR(n1, n2, 0.5);
    EXPECT_NEAR(u1, u2, 0.5);
}
