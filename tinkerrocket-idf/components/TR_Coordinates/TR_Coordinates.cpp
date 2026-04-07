#include "TR_Coordinates.h"

TR_Coordinates::TR_Coordinates()
{
    a = 6378137.0;            // Semi-major axis
    f = 1.0 / 298.257223563;  // Flattening
    e2 = 2 * f - f * f;       // Eccentricity squared
}

void TR_Coordinates::setENUOrigin(double lat_deg_in,
                                  double lon_deg_in,
                                  double alt_in)
{
    // Store in radians for consistent trig usage
    lat0 = lat_deg_in * DEG2RAD;
    lon0 = lon_deg_in * DEG2RAD;
    alt0 = alt_in;

    geodeticToECEF(lat0,
                   lon0,
                   alt0,
                   x0,
                   y0,
                   z0);
}

void TR_Coordinates::setENUOriginECEF(double x_ecef,
                                      double y_ecef,
                                      double z_ecef)
{
    double lat_deg, lon_deg;
    ecefToGeodetic(x_ecef, y_ecef, z_ecef, lat_deg, lon_deg, alt0);

    // ecefToGeodetic outputs degrees; store as radians
    lat0 = lat_deg * DEG2RAD;
    lon0 = lon_deg * DEG2RAD;

    x0 = x_ecef;
    y0 = y_ecef;
    z0 = z_ecef;
}

// Convert ECEF (X, Y, Z) to Geodetic (Lat, Lon, Alt)
void TR_Coordinates::ecefToGeodetic(double x, 
                                    double y, 
                                    double z,
                                    double &lat,
                                    double &lon,
                                    double &alt)
{    
    // Compute longitude
    lon = atan2(y, x);
    
    // Compute distance from Z-axis
    double p = sqrt(x * x + y * y);

    // Initial estimate of latitude
    lat = atan2(z, p * (1 - e2));
    double N, prev_lat;

    // Iterate to refine latitude and altitude
    // Converge to ~0.1 mm accuracy
    do {
        prev_lat = lat;
        N = a / sqrt(1 - e2 * pow(sin(lat), 2)); // Prime vertical radius
        alt = p / cos(lat) - N;
        lat = atan2(z + N * e2 * sin(lat), p);
    } while (fabs(lat - prev_lat) > 1e-10); 

    // Convert to degrees
    lat *= RAD2DEG;
    lon *= RAD2DEG;

}


void TR_Coordinates::geodeticToENU(double lat_deg,
                                   double lon_deg,
                                   double alt,
                                   double& e,
                                   double& n,
                                   double& u)
{
    double x, y, z;
    double lat_rad = lat_deg * DEG2RAD;
    double lon_rad = lon_deg * DEG2RAD;
    geodeticToECEF(lat_rad, lon_rad, alt, x, y, z);
    ECEFToENU(x,y,z,e,n,u);
}

void TR_Coordinates::geodeticToECEF(double lat, 
                                    double lon,
                                    double alt,
                                    double& x,
                                    double& y,
                                    double& z)
{
    double sinLat = sin(lat);
    double cosLat = cos(lat);
    double sinLon = sin(lon);
    double cosLon = cos(lon);

    double N = a / sqrt(1 - e2 * sinLat * sinLat);

    x = (N + alt) * cosLat * cosLon;
    y = (N + alt) * cosLat * sinLon;
    z = (N * (1 - e2) + alt) * sinLat;     
}


void TR_Coordinates::ECEFToENU(double x, 
                               double y, 
                               double z, 
                               double& e, 
                               double& n, 
                               double& u) 
{
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    double sinLat0 = sin(lat0);
    double cosLat0 = cos(lat0);
    double sinLon0 = sin(lon0);
    double cosLon0 = cos(lon0);

    e = -sinLon0 * dx + cosLon0 * dy;
    n = -sinLat0 * cosLon0 * dx - sinLat0 * sinLon0 * dy + cosLat0 * dz;
    u = cosLat0 * cosLon0 * dx + cosLat0 * sinLon0 * dy + sinLat0 * dz;
}

// Quaternion to Euler angles (3-2-1)
void TR_Coordinates::Quat2Euler(float quat[4], float euler[3])
{
    float m11 = 2*(quat[0]*quat[0] + quat[1]*quat[1]) - 1;
    float m12 = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
    float m13 = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
    float m23 = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
    float m33 = 2*(quat[0]*quat[0] + quat[3]*quat[3]) - 1;

    euler[2] = atan2(m12, m11);
    float sinp = -m13;
    if (fabsf(sinp) >= 1.0f)
        euler[1] = copysignf((float)M_PI / 2.0f, sinp);
    else
        euler[1] = asinf(sinp);
    euler[0] = atan2(m23, m33);
}

